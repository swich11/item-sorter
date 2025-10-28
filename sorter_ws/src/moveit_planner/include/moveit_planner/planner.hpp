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
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/hybrid_planning_manager/planner_logic_interface.h>


namespace moveit::hybrid_planning 
{


class PlannerLogic : public PlannerLogicInterface {
public:
    PlannerLogic() = default;
    ~PlannerLogic() = default;

    // The plugin needs a shared pointer to the hybrid planning manager to access its member functions like planGlobalTrajectory()
    bool initialize(const std::shared_ptr<HybridPlanningManager>& hybrid_planning_manager) override;

    // This function can be used to implement reaction to some default Hybrid Planning events
    ReactionResult react(const HybridPlanningEvent& event) override;

    // Here are reactions to custom events encoded as string implemented
    ReactionResult react(const std::string& event) override;

private:
    std::shared_ptr<HybridPlanningManager> hybrid_planning_manager;
};
};




class Planner : public rclcpp::Node {
    public:
        Planner();



    private:
        moveit_msgs::msg::CollisionObject generateCollisionObject(
            float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id);


        geometry_msgs::msg::Pose generatePoseMsg(
            float x,float y, float z,float qx,float qy,float qz,float qw
        );

        


        void objectDetectedCallback(); // Add or modify collision objects for detected objects
        void generatePathCallback(); // Generate and execute the path

        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
};