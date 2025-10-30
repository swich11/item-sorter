#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <string>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <geometry_msgs/msg/pose.hpp>


#include "interfaces/srv/move.hpp"


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


        bool move(std::shared_ptr<interfaces::srv::Move::Response> res,
                  const geometry_msgs::msg::Pose &target_pose);

        
        bool grasp();


        bool ungrasp();


        void setPathConstraints();


        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

        rclcpp::Service<interfaces::srv::Move>::SharedPtr move_server;
        geometry_msgs::msg::Pose home;
};