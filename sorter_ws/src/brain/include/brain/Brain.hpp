#include <map>
#include <string>
#include <vector>


#include "rclcpp/rclcpp.hpp"


#include "interfaces/msg/labelled_pose_array.hpp"
#include "interfaces/msg/labelled_pose.hpp"
#include "interfaces/srv/move.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class Brain : public rclcpp::Node {
public:
    Brain();

private:
    void item_topic_callback(const interfaces::msg::LabelledPoseArray &msg);

    void goal_topic_callback(const interfaces::msg::LabelledPoseArray &msg);

    void send_move_request(const std::string &label);

    void move_request_response(const std::string &label, 
                               rclcpp::Client<interfaces::srv::Move>::SharedFuture future);


    std::map<std::string, geometry_msgs::msg::Pose> item_pose_map;
    std::map<std::string, geometry_msgs::msg::PoseStamped> goal_pose_map;

    rclcpp::Client<interfaces::srv::Move>::SharedPtr move_client;
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr item_pose_subscription;
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr goal_pose_subscription;
};