#include "brain/Brain.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


Brain::Brain() : Node("brain") {
    move_client = this->create_client<interfaces::srv::Move>("/moveit_planner/move");
    transform_client = this->create_client<interfaces::srv::TransformLookupArray>("/pose_lookup_array");
    item_pose_subscription = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/camera/objects/labelled_pose_array", 10, std::bind(&Brain::object_topic_callback, this, _1)
    );
    pose_update_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/brain/move/pose", 10
    );
    move_call_thread = std::thread(&Brain::spin_wait_for_items, this);
    running = true;

    RCLCPP_INFO(this->get_logger(), "Brain Node Started.");
}


Brain::~Brain() {
    if (move_call_thread.joinable()) {
        this->running = false;
        item_queue_sem.release();
        move_call_thread.join();
        RCLCPP_INFO(this->get_logger(), "Closing item handler thread.");
    }
}


void Brain::object_topic_callback(const interfaces::msg::LabelledPoseArray &msg) {
    transform_labelled_pose_array(msg, std::bind(&Brain::update_item_map, this, _1));
}


void Brain::transform_labelled_pose_array(const interfaces::msg::LabelledPoseArray &msg,
                                          std::function<void(const interfaces::msg::LabelledPoseArray&)> f_update_map) {
    auto req = std::make_shared<interfaces::srv::TransformLookupArray::Request>();
    req->poses.resize(msg.poses.size());
    std::transform(msg.poses.begin(), msg.poses.end(), req->poses.begin(), 
        [&msg](const interfaces::msg::LabelledPose &l_pose) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose = l_pose.pose;
            return pose;
        }
    );
    req->to_link = "tool0";
    RCLCPP_INFO(this->get_logger(), "sending transform request.");
    transform_client->async_send_request(req, 
        [this, f_update_map, &msg](rclcpp::Client<interfaces::srv::TransformLookupArray>::SharedFuture future) {
            auto res = future.get();
            if (!res->success) {
                RCLCPP_INFO(this->get_logger(), "Transform lookup failed.");
                return;
            }
            // Copy transformed poses to tf_msg
            interfaces::msg::LabelledPoseArray tf_msg;
            tf_msg.header = msg.header;
            int len_poses = res->poses.size();
            for (int i = 0; i < len_poses; i++) {
                interfaces::msg::LabelledPose pose = msg.poses[i];
                pose.pose = res->poses[i].pose;
                tf_msg.poses.push_back(pose);
                RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tf_msg.poses[i].pose.position.x,
                                                                    tf_msg.poses[i].pose.position.y,
                                                                    tf_msg.poses[i].pose.position.z);
            }
            // TODO: filter poses that are not in robot range.
            // Do update step
            f_update_map(tf_msg);
        }
    );
}


void Brain::update_item_map(const interfaces::msg::LabelledPoseArray &msg) {
    // sort buckets and items
    interfaces::msg::LabelledPoseArray item_poses;
    interfaces::msg::LabelledPoseArray goal_poses;
    for (auto pose : msg.poses) {
        if (pose.label.find("Bucket") != std::string::npos) {
            goal_poses.poses.push_back(pose);
        } else {
            item_poses.poses.push_back(pose);
        }
    }

    // Add goal poses to map
    for (auto goal_pose : goal_poses.poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg.header;
        pose_stamped.pose = goal_pose.pose;
        goal_pose_map[goal_pose.label] = pose_stamped;
    }

    // Add item poses to map and dispatch
    for (auto item_pose : item_poses.poses) {
        try {
            item_pose_map.at(item_pose.label).pose.pose = item_pose.pose;
            item_pose_map[item_pose.label].pose.header = msg.header;
        }
        catch (std::out_of_range const&) {
            item_pose_map[item_pose.label].pose.pose = item_pose.pose;
            item_pose_map[item_pose.label].pose.header = msg.header;
            item_pose_map[item_pose.label].in_queue = false;
        }
        // Queue item to be moved to goal
        try {
            // check if goal is found and recent, queue when it is and release semaphore.
            auto& goal_pose = goal_pose_map.at(get_goal_label(item_pose.label));
            if (!item_pose_map[item_pose.label].in_queue
                    && rclcpp::Time(msg.header.stamp) - rclcpp::Time(goal_pose.header.stamp) 
                        < rclcpp::Duration(1, 0)) {
                item_queue.push(item_pose.label);
                item_pose_map[item_pose.label].in_queue = true;
                item_queue_sem.release(); // Release semaphore for movement goal
            }
        }
        catch (std::out_of_range const&) {}
    }

    // TODO: dequeue items and goals that are not recent
}


// Waits for items to be added to the queue and sends the move request once they are added.
void Brain::spin_wait_for_items() {
    while (true) {
        item_queue_sem.acquire();
        if (!this->running) {
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for item semaphore.");
        auto item_label = item_queue.front();
        send_move_request(item_label);
        item_queue.pop();
    }
}


void Brain::send_move_request(const std::string &item_label) {
    auto req = std::make_shared<interfaces::srv::Move::Request>();
    req->grasp = true;
    this->timer = this->create_wall_timer(50ms, 
        std::function<void()>(std::bind(&Brain::publish_item_pose, this, std::cref(item_label))));
    RCLCPP_INFO(this->get_logger(), "Sending Move Request %s", item_label.c_str());
    
    // Sends a request to the moveit planner which performs moves continuously.
    // the moveit client will cancel if it can no longer see the object

    // Go to item -> grab -> go to goal -> grab -> loop
    move_client->wait_for_service(100ms); // Don't pre-empt the service call
    move_client->async_send_request(req,
        [this, &item_label, &req](rclcpp::Client<interfaces::srv::Move>::SharedFuture future) {
            if (this->move_request_response(future)) {
                // Send goal as request now
                this->timer = this->create_wall_timer(50ms,
                    std::function<void()>(std::bind(&Brain::publish_goal_pose, this, get_goal_label(item_label)))
                );
                req->grasp = false;
                move_client->wait_for_service(100ms); // Wait for service to not pre-empt service call
                move_client->async_send_request(req,
                    [this](rclcpp::Client<interfaces::srv::Move>::SharedFuture future) {
                        if (this->move_request_response(future)) {
                            // TODO: add error handling here.
                        }

                        // add drop item request on goal move failure?
                    }
                );
                item_pose_map.erase(item_label);
            } else {
                // TODO: more error handling.

                // Allow item to be queued again on failure
                item_pose_map[item_label].in_queue = false;
            }
        }
    );
}


bool Brain::move_request_response(rclcpp::Client<interfaces::srv::Move>::SharedFuture future) {
    this->timer->cancel();
    auto res = future.get();
    if (res->success) {
        return true;
    }
    RCLCPP_INFO(this->get_logger(), res->message.c_str());
    return false;
}


// Publish stamped poses
inline void Brain::publish_goal_pose(const std::string &label) {
    pose_update_publisher->publish(goal_pose_map[label]);
} 


inline void Brain::publish_item_pose(const std::string &label) {
    pose_update_publisher->publish(item_pose_map[label].pose);
}


std::string Brain::get_goal_label(const std::string &item_label) {
    std::string goal_label = "Bucket";
    if (item_label.find("Square") != std::string::npos) {
        goal_label = "SquareBucket";
    } else if (item_label.find("Circle") != std::string::npos) {
        goal_label = "CircleBucket";
    } else if (item_label.find("Hexagon") != std::string::npos) {
        goal_label = "HexagonBucket";
    }
    return goal_label;
}


void Brain::send_move_request(const geometry_msgs::msg::PoseStamped &pose) {
    auto req = std::make_shared<interfaces::srv::Move::Request>();
    req->grasp = false;
    this->timer = this->create_wall_timer(50ms,
        [this, &pose] {
            pose_update_publisher->publish(pose);
        }
    );
    RCLCPP_INFO(this->get_logger(), "Sending Move Request Debug");
    move_client->wait_for_service(2s);
    move_client->async_send_request(req,
        [this](rclcpp::Client<interfaces::srv::Move>::SharedFuture future) {
            this->move_request_response(future);
        }
    );
}
   
































int test_pose_callbacks(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto brain = std::make_shared<Brain>();

    interfaces::msg::LabelledPose l_pose;
    l_pose.colour = "orange";
    l_pose.label = "red1";
    l_pose.shape = "square";
    l_pose.pose.position.x = 0.0;
    l_pose.pose.position.x = 0.0;
    l_pose.pose.position.x = 0.0;
    l_pose.pose.orientation.w = 1.0;
    l_pose.pose.orientation.x = 0.0;
    l_pose.pose.orientation.y = 0.0;
    l_pose.pose.orientation.z = 0.0;

    interfaces::msg::LabelledPoseArray l_array;
    l_array.header.frame_id = "camera_link";
    // l_array.header.stamp = brain->get_clock()->now();

    l_array.poses.push_back(l_pose);


    // brain->item_topic_callback(l_array);
    // brain->goal_topic_callback(l_array);

    rclcpp::spin(brain);
    rclcpp::shutdown();
    return 0;
}


// Main functions
int debug_main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto brain = std::make_shared<Brain>();
    geometry_msgs::msg::Pose pose;

    pose.orientation.w = 0;
    pose.orientation.x = 1;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    pose.position.x = 0.56;
    pose.position.y = 0.35;
    pose.position.z = 0.17;

    geometry_msgs::msg::PoseStamped p_s;
    p_s.pose = pose;

    brain->send_move_request(p_s);

    rclcpp::spin(brain);
    rclcpp::shutdown();
    return 0;
} 

int release_main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2
    );
    auto brain = std::make_shared<Brain>();
    executor.add_node(brain);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char* argv[]) {
    return release_main(argc, argv);
}