#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <geometry_msgs/msg/pose.hpp>


#include "interfaces/srv/move.hpp"


constexpr double POSITION_PRECISION = 0.01; // 1 cm
constexpr double ORIENTATION_PRECISION = 5.0/180.0 * M_PI; // 5 degrees



class Planner : public rclcpp::Node {
    public:
        Planner();

    private:
        moveit_msgs::msg::CollisionObject generateCollisionObject(
            float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id);

        geometry_msgs::msg::Pose generatePoseMsg(
            float x,float y, float z,float qx,float qy,float qz,float qw
        );

        void moveServiceCallback(const std::shared_ptr<interfaces::srv::Move::Request> req,
                                 std::shared_ptr<interfaces::srv::Move::Response> res);

        void goalPoseCallback(const geometry_msgs::msg::Pose &pose);

        bool move(std::shared_ptr<interfaces::srv::Move::Response> res);

        void asyncMoveHome();

        bool grasp();

        bool ungrasp();

        void setPathConstraints();

        bool isPoseClose(const geometry_msgs::msg::Pose &a,
                         const geometry_msgs::msg::Pose &b);

        inline double norm(const geometry_msgs::msg::Point &a,
                    const geometry_msgs::msg::Point &b);

        inline double norm(const geometry_msgs::msg::Quaternion &a,
                    const geometry_msgs::msg::Quaternion &b);

    private:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pose_subscription;
        rclcpp::Service<interfaces::srv::Move>::SharedPtr move_server;
        geometry_msgs::msg::Pose goal_pose;
        geometry_msgs::msg::Pose home_pose;
        bool grabbed_home_pose;
};