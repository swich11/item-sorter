#include "transforms/TransformNode.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


TransformNode::TransformNode() : Node("transform_node") {
    lookup_service = this->create_service<interfaces::srv::TransformLookup>(
        "/pose_lookup", std::bind(&TransformNode::lookup_service_callback, this, _1, _2)
    );
    lookup_array_service = this->create_service<interfaces::srv::TransformLookupArray>(
        "/pose_lookup_array", std::bind(&TransformNode::lookup_array_service_callback, this, _1, _2)
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    broadcast_static_camera_transform();
}


void TransformNode::broadcast_static_camera_transform() {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.frame_id = "base_link"; // Parent Frame
    tf.header.stamp = this->get_clock()->now();

    tf.child_frame_id = "camera_link"; // Child Frame
    
    float angle_halved = 45.0 * M_PI_2 / 180.0; // Pitch Angle
    tf2::Quaternion z_rot = tf2::Quaternion(0.0, 0.0, 1.0, 0.0);
    tf2::Quaternion y_rot = tf2::Quaternion(0.0, -sin(angle_halved), 0.0, cos(angle_halved));

    tf.transform.rotation = tf2::toMsg(y_rot * z_rot); // rotate yaw first, then pitch

    tf.transform.translation.x = 1.4;
    tf.transform.translation.y = 0.0174152;
    tf.transform.translation.z = 0.72;

    tf.transform.rotation.x = -0.388123;
    tf.transform.rotation.y = -0.0054127;
    tf.transform.rotation.z = 0.92155;
    tf.transform.rotation.w = 0.0087602;

    tf_static_broadcaster_->sendTransform(tf);
}


void TransformNode::lookup_service_callback(
    const std::shared_ptr<interfaces::srv::TransformLookup::Request> req,
    const std::shared_ptr<interfaces::srv::TransformLookup::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received lookup request.");

    try {
        auto transform = tf_buffer_->lookupTransform(req->to_link, 
                                                    req->pose.header.frame_id, 
                                                    tf2::TimePointZero,
                                                    500ms);
        tf2::doTransform(req->pose, res->pose, transform);
        RCLCPP_INFO(this->get_logger(), "Transform successful.");
        res->success = true;
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not lookup transform %s", e.what());
        res->success = false;
    }
}


void TransformNode::lookup_array_service_callback(const std::shared_ptr<interfaces::srv::TransformLookupArray::Request> req,
                                                  const std::shared_ptr<interfaces::srv::TransformLookupArray::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received lookup array request.");

    try {
        auto transform = tf_buffer_->lookupTransform(req->to_link, 
                                                    req->poses[0].header.frame_id, 
                                                    tf2::TimePointZero,
                                                    500ms);
        // Apply transform to each pose
        geometry_msgs::msg::PoseStamped output_pose;
        for (auto pose : req->poses) {
            tf2::doTransform(pose, output_pose, transform);
            res->poses.push_back(output_pose);
        }
        RCLCPP_INFO(this->get_logger(), "Transforms successful.");
        res->success = true;
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not lookup transform %s", e.what());
        res->success = false;
    }
}



int test(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto lookup = std::make_shared<TransformNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lookup);
    for (int i = 0; i < 10; i++) {
        auto req = std::make_shared<interfaces::srv::TransformLookup::Request>();
        auto res = std::make_shared<interfaces::srv::TransformLookup::Response>();
        req->pose.header.stamp = lookup->get_clock()->now();
        req->pose.header.frame_id = "base_link";
        req->pose.pose.position.x = 0.0;
        req->pose.pose.position.y = 0.0;
        req->pose.pose.position.z = 0.0;
        req->pose.pose.orientation.w = 1.0;
        req->pose.pose.orientation.x = 0.0;
        req->pose.pose.orientation.y = 0.0;
        req->pose.pose.orientation.z = 0.0;
        req->to_link = "tool0";
        lookup->lookup_service_callback(req, res);
        
        std::ostringstream stream;
        stream << "frame id: " << res->pose.header.frame_id << std::endl
            << "position: " << "x: " << res->pose.pose.position.x
            << " y: " << res->pose.pose.position.y
            << " z: " << res->pose.pose.position.z << std::endl
            << "orientation: " << " w: " << res->pose.pose.orientation.w
            << " x: " << res->pose.pose.orientation.x
            << " y: " << res->pose.pose.orientation.y
            << " z: " << res->pose.pose.orientation.z << std::endl;
        
        RCLCPP_INFO(lookup->get_logger(), stream.str().c_str());
        executor.spin_some();
    }
    rclcpp::shutdown();

    return 0;
}


int run(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto lookup = std::make_shared<TransformNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lookup);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}


int main(int argc, char* argv[]) {
    return run(argc, argv);
}