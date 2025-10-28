#include "moveit_planner/planner.hpp"


Planner::Planner() : Node("planner") {
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    planning_scene_monitor = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(std::shared_ptr<rclcpp::Node>(this), "robot_description");

    std::string frame_id = move_group_interface->getPlanningFrame();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {
        generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"),
        generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"),
        generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.02, frame_id, "table"),
    };
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    // Setup allowed collisions
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);
    auto& acm = planning_scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry("table", "base_link_inertia", true);
}


moveit_msgs::msg::CollisionObject Planner::generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
}