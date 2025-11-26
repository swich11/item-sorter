#include <map>
#include <string>
#include <vector>
#include <queue>
#include <chrono>


#include "rclcpp/rclcpp.hpp"


#include "interfaces/msg/labelled_pose_array.hpp"
#include "interfaces/msg/labelled_pose.hpp"
#include "interfaces/srv/move.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


struct ItemPose {
    geometry_msgs::msg::Pose pose;
    bool in_queue;
};


class Brain : public rclcpp::Node {
public:
    Brain();

    // Debugging function
    void send_move_request(const geometry_msgs::msg::Pose &pose);

private:
    void item_topic_callback(const interfaces::msg::LabelledPoseArray &msg);

    void goal_topic_callback(const interfaces::msg::LabelledPoseArray &msg);

    void send_move_request(const std::string &label);

    bool move_request_response(rclcpp::Client<interfaces::srv::Move>::SharedFuture future);

    inline void publish_goal_pose(const std::string &label);

    inline void publish_item_pose(const std::string &label);

    std::string get_goal_label(const std::string &item_label);
        

    std::queue<std::string> item_queue;
    std::map<std::string, ItemPose> item_pose_map;
    std::map<std::string, geometry_msgs::msg::PoseStamped> goal_pose_map;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<interfaces::srv::Move>::SharedPtr move_client;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_update_publisher;
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr item_pose_subscription;
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr goal_pose_subscription;
};