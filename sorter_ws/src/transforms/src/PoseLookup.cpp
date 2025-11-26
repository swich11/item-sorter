#include "transforms/PoseLookup.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


PoseLookup::PoseLookup() : Node("pose_lookup") {
    lookup_service = this->create_service<interfaces::srv::TransformLookup>(
        "/pose_lookup", std::bind(&PoseLookup::lookup_service_callback, this, _1, _2));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void PoseLookup::lookup_service_callback(
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


int test(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto lookup = std::make_shared<PoseLookup>();
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
    auto lookup = std::make_shared<PoseLookup>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lookup);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}


int main(int argc, char* argv[]) {
    return test(argc, argv);
}