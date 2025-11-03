#include "moveit_planner/Planner.hpp"


using std::placeholders::_1;
using std::placeholders::_2;



Planner::Planner() : Node("planner") {
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);
    move_group_interface->allowReplanning(true);
    move_group_interface->startStateMonitor(3.0);


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

    setPathConstraints();

    grabbed_home_pose = false;
    move_server = this->create_service<interfaces::srv::Move>("/moveit_planner/move", std::bind(&Planner::moveServiceCallback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Planner Launched. Ready for Commands");
}

void Planner::moveServiceCallback(const std::shared_ptr<interfaces::srv::Move::Request> req,
                                  std::shared_ptr<interfaces::srv::Move::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Move Request.");
    if (!grabbed_home_pose) {
        home_pose = move_group_interface->getCurrentPose().pose;
        grabbed_home_pose = true;
    }
    move_group_interface->stop();
    move(res, req->start_pose);
    RCLCPP_INFO(this->get_logger(), "At Start Pose.");
    grasp();
    move(res, req->goal_pose);
    RCLCPP_INFO(this->get_logger(), "At Goal Pose.");
    ungrasp();
    asyncMoveHome();
}

bool Planner::move(std::shared_ptr<interfaces::srv::Move::Response> res,
                   const geometry_msgs::msg::Pose &target_pose) {
    if (move_group_interface->setPoseTarget(target_pose)) {
        auto ret = move_group_interface->move();
        if (ret == moveit::core::MoveItErrorCode::SUCCESS) {
            res->success = true;
        } else {
            res->success = false;
            res->message = "Move Failed: " + moveit::core::error_code_to_string(ret);
            return false;
        }
    } else {
        return false;
    }
    return true;
}

void Planner::asyncMoveHome() {
    move_group_interface->setPoseTarget(home_pose);
    move_group_interface->asyncMove();
}

// TODO: Add grasping
bool Planner::grasp() {
    sleep(3);
    return true;
}

bool Planner::ungrasp() {
    sleep(3);
    return true;
}

void Planner::setPathConstraints() {
    // Lock wrist 2 link to simplify path planning
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::JointConstraint wrist_2_constraint;
    wrist_2_constraint.joint_name = "wrist_2_link";
    wrist_2_constraint.position = -M_PI / 2;
    wrist_2_constraint.tolerance_above = 0.05;
    wrist_2_constraint.tolerance_below = 0.05;
    wrist_2_constraint.weight = 0;
    constraints.joint_constraints.push_back(wrist_2_constraint);
    // move_group_interface->setPathConstraints(constraints);
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

geometry_msgs::msg::Pose Planner::generatePoseMsg(float x, float y, float z, float qx, float qy, float qz, float qw) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = qw;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<Planner>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner);
    executor.spin();
    rclcpp::shutdown();
}