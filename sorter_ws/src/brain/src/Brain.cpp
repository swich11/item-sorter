#include "Brain.hpp"


using std::placeholders::_1;


Brain::Brain() : Node("brain") {
    move_client = this->create_client<interfaces::srv::Move>("move");
    item_pose_subscription = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/base/objects/labelled_pose_array", 10, std::bind(&Brain::item_topic_callback, this, _1)
    );
    goal_pose_subscription = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/base/goals/labelled_pose_array", 10, std::bind(&Brain::goal_topic_callback, this, _1)
    );
}


void Brain::item_topic_callback(const interfaces::msg::LabelledPoseArray &msg) {
    for (auto item_pose : msg.poses) {
        item_pose_map[item_pose.label] = item_pose.pose;
        try {
            auto& goal_pose = goal_pose_map.at(item_pose.label);
            if (rclcpp::Time(msg.header.stamp) - rclcpp::Time(goal_pose.header.stamp) < rclcpp::Duration(1, 0)) {
                queue_move_request(item_pose.label);
            }
        }
        catch (std::out_of_range e) {}
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


void Brain::queue_move_request(const std::string &label) {

}


void Brain::send_move_request(const std::string &label) {
    auto req = std::make_shared<interfaces::srv::Move::Request>();
    req->goal_pose = goal_pose_map.at(label).pose;
    req->start_pose = item_pose_map.at(label);
    move_client->async_send_request(req, std::bind(&Brain::move_request_response, this, _1));
}