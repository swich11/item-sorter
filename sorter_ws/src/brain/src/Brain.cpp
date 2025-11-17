#include "brain/Brain.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


Brain::Brain() : Node("brain") {
    move_client = this->create_client<interfaces::srv::Move>("/moveit_planner/move");
    item_pose_subscription = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/base/objects/labelled_pose_array", 10, std::bind(&Brain::item_topic_callback, this, _1)
    );
    goal_pose_subscription = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/base/goals/labelled_pose_array", 10, std::bind(&Brain::goal_topic_callback, this, _1)
    );
    pose_update_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
        "/brain/move/pose", 10
    );

    RCLCPP_INFO(this->get_logger(), "Brain Node Started.");
}


void Brain::item_topic_callback(const interfaces::msg::LabelledPoseArray &msg) {
    for (auto item_pose : msg.poses) {
        // Update item pose in the map
        try {
            item_pose_map.at(item_pose.label).pose = item_pose.pose;
        }
        catch (std::out_of_range const&) {
            item_pose_map[item_pose.label].pose = item_pose.pose;
            item_pose_map[item_pose.label].in_queue = false;
        }
        // Queue item to be moved to goal
        try {
            auto& goal_pose = goal_pose_map.at(get_goal_label(item_pose.label));
            // only queue once goal is found and is recent
            if (!item_pose_map[item_pose.label].in_queue
                    && rclcpp::Time(msg.header.stamp) - rclcpp::Time(goal_pose.header.stamp) 
                        < rclcpp::Duration(1, 0)) {
                item_queue.push(item_pose.label);
                item_pose_map[item_pose.label].in_queue = true;
            }
        }
        catch (std::out_of_range const&) {}
    }
}


void Brain::goal_topic_callback(const interfaces::msg::LabelledPoseArray &msg) {
    for (auto pose : msg.poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg.header;
        pose_stamped.pose = pose.pose;
        goal_pose_map[pose.label] = pose_stamped;
    }
}


void Brain::send_move_request(const std::string &item_label) {
    auto req = std::make_shared<interfaces::srv::Move::Request>();
    req->grasp = true;
    this->timer = this->create_wall_timer(50ms, 
        std::function<void()>(std::bind(&Brain::publish_item_pose, this, std::cref(item_label))));
    RCLCPP_INFO(this->get_logger(), "Sending Move Request %s", item_label.c_str());

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
                        this->move_request_response(future);
                        // add drop item request on goal move failure?
                    }
                );
                item_pose_map.erase(item_label);
            } else {
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


inline void Brain::publish_goal_pose(const std::string &label) {
    pose_update_publisher->publish(goal_pose_map[label].pose);
} 


inline void Brain::publish_item_pose(const std::string &label) {
    pose_update_publisher->publish(item_pose_map[label].pose);
}


std::string Brain::get_goal_label(const std::string &item_label) {
    std::string goal_label = item_label;
    for(int i = 0; i < static_cast<int>(item_label.length()); i++) {
        if (isdigit(item_label[i])) {
            goal_label = item_label.substr(0, i);
        }
    }
    return goal_label;
}


void Brain::send_move_request(const geometry_msgs::msg::Pose &pose) {
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


    brain->send_move_request(pose);

    rclcpp::spin(brain);
    rclcpp::shutdown();
    return 0;
} 

int release_main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Brain>());
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char* argv[]) {
    return debug_main(argc, argv);
}