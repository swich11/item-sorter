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
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "interfaces/srv/move.hpp"


constexpr double POSITION_PRECISION = 0.01; // 1 cm
constexpr double ORIENTATION_PRECISION = 5.0/180.0 * M_PI; // 5 degrees
constexpr double GRIPPER_HEIGHT = 0.16;
constexpr double GRIPPER_OFFSET = 0.1;
constexpr double GRAB_OFFSET = 0.03;


class Planner : public rclcpp::Node {
    public:
        Planner();

        void initialiseMoveIt();

    private:
        moveit_msgs::msg::CollisionObject generateCollisionObject(
            float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id);

        geometry_msgs::msg::Pose generatePoseMsg(
            float x,float y, float z,float qx,float qy,float qz,float qw
        );

        void moveServiceCallback(std::shared_ptr<interfaces::srv::Move::Request> req,
                                 std::shared_ptr<interfaces::srv::Move::Response> res);

        void goalPoseCallback(const geometry_msgs::msg::PoseStamped &pose);

        void cancelMoveCallback(const std_msgs::msg::Empty&);

        void jointStateCallback(const sensor_msgs::msg::JointState &joint_state);

        bool move(std::shared_ptr<interfaces::srv::Move::Response> res, bool grasp, geometry_msgs::msg::Pose goal);

        void asyncMoveHome();

        void grasp();

        void ungrasp();

        moveit_msgs::msg::Constraints getPathConstraints();

        moveit_msgs::msg::Constraints getGripPathConstraints();

        bool isPoseClose(const geometry_msgs::msg::Pose &a,
                         const geometry_msgs::msg::Pose &b);

        inline double norm(const geometry_msgs::msg::Point &a,
                    const geometry_msgs::msg::Point &b);

        inline double norm(const geometry_msgs::msg::Quaternion &a,
                    const geometry_msgs::msg::Quaternion &b);

    private:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

        rclcpp::CallbackGroup::SharedPtr planner_callback_group;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_move_subscription;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;
        rclcpp::Service<interfaces::srv::Move>::SharedPtr move_server;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_pub;    // publisher to send commands to Arduino
        
        geometry_msgs::msg::PoseStamped goal_pose;
        geometry_msgs::msg::Pose home_pose;
        bool grabbed_home_pose;
        bool move_canceled;
        bool received_joint_states;

        moveit_msgs::msg::Constraints regular_constraints;
        std::map<std::string, double> last_joint_values;
};