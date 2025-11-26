#include "transforms/PoseLookup.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


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
        auto transform = tf_buffer_->lookupTransform(
                            req->to_link, req->pose.header.frame_id, tf2::TimePointZero);           
        tf2::doTransform(req->pose, res->pose, transform);
        RCLCPP_INFO(this->get_logger(), "Transform successful.");
        res->success = true;
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not lookup transform %s", e.what());
        res->success = false;
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseLookup>());
    rclcpp::shutdown();
    return 0;
}