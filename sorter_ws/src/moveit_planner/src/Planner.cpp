#include "moveit_planner/Planner.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


// TODO: make grasp handle case where new poses aren't being streamed correctly.



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
    goal_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/brain/move/pose", 10, std::bind(&Planner::goalPoseCallback, this, _1));
    move_server = this->create_service<interfaces::srv::Move>("/moveit_planner/move", std::bind(&Planner::moveServiceCallback, this, _1, _2));
    arduino_pub = this->create_publisher<std_msgs::msg::String>("/arduino_cmds", 10);   // initialize publisher to send commands to Arduino
    RCLCPP_INFO(this->get_logger(), "Planner Launched. Ready for Commands");
}

void Planner::moveServiceCallback(std::shared_ptr<interfaces::srv::Move::Request> req,
                                  std::shared_ptr<interfaces::srv::Move::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Move Request.");
    if (!grabbed_home_pose) {
        // Initialise the home pose
        home_pose = move_group_interface->getCurrentPose().pose;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, w: %f", home_pose.orientation.x, 
                                                                      home_pose.orientation.y,
                                                                      home_pose.orientation.z,
                                                                      home_pose.orientation.w);
        home_pose.orientation.x = sqrt(2) / 2.0;
        home_pose.orientation.y = -sqrt(2) / 2.0;
        home_pose.orientation.z = 0.0;
        home_pose.orientation.w = 0.0;
        grabbed_home_pose = true;
        
        // For now unless we add pose rotation
        goal_pose.pose.orientation = home_pose.orientation;
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.header.frame_id = "tool0";
    }
    move_group_interface->stop();
    move(res, req->grasp);
    RCLCPP_INFO(this->get_logger(), "At Goal Pose.");
}


bool Planner::move(std::shared_ptr<interfaces::srv::Move::Response> res, bool grasp) {
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, w: %f", goal_pose.pose.orientation.x, 
                                                                  goal_pose.pose.orientation.y,
                                                                  goal_pose.pose.orientation.z,
                                                                  goal_pose.pose.orientation.w);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::msg::Pose tracked_goal = home_pose; // Anything different to the goal_pose works
    do {
        if (!isPoseClose(tracked_goal, goal_pose.pose)) {
            // Change goal when the object moves
            tracked_goal = goal_pose.pose;
            move_group_interface->setPoseTarget(tracked_goal);
            move_group_interface->stop();
            auto ret = move_group_interface->plan(plan);
            if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Planning Failed :(.");
                res->success = false;
                RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                       tracked_goal.position.y,
                                                                       tracked_goal.position.z);
                res->message = "Planning failed during move: " + moveit::core::error_code_to_string(ret);
                return false;
            }
            move_group_interface->asyncExecute(plan);
        }
        rclcpp::sleep_for(50ms);
    } while (!move_group_interface->getMoveGroupClient().action_server_is_ready());
    // Pose is close to tracked goal.

    // drop from height
    if (!grasp) {
        this->ungrasp();
        res->success = true;
        return true;
    }
    // attempt grasp
    auto grab_pose = goal_pose.pose;
    grab_pose.position.z = GRAB_OFFSET;
    move_group_interface->setPoseTarget(goal_pose.pose);
    move_group_interface->stop();
    auto ret = move_group_interface->plan(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Planning Failed :(.");
        res->success = false;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                tracked_goal.position.y,
                                                                tracked_goal.position.z);
        res->message = "Planning failed during grab: " + moveit::core::error_code_to_string(ret);
        return false;
    }
    move_group_interface->execute(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "Execute Failed :(.");
        res->success = false;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                tracked_goal.position.y,
                                                                tracked_goal.position.z);
        res->message = "Execute failed during grab: " + moveit::core::error_code_to_string(ret);
        return false;
    }
    this->grasp();

    res->success = true;
    return true;
}

void Planner::asyncMoveHome() {
    move_group_interface->setPoseTarget(home_pose);
    move_group_interface->asyncMove();
}

// sends serial mesg "close" over port to teensy
void Planner::grasp() {
    std_msgs::msg::String msg;
    msg.data = "close\n";
    arduino_pub->publish(msg);
}

// sends serial msg "open" over port to teensy
void Planner::ungrasp() {
    std_msgs::msg::String msg;
    msg.data = "open\n";
    arduino_pub->publish(msg);
}

void Planner::setPathConstraints() {
    // Lock wrist 2 link to simplify path planning
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::JointConstraint wrist_2_constraint;
    wrist_2_constraint.joint_name = "wrist_2_joint";
    wrist_2_constraint.position = -M_PI / 2;
    wrist_2_constraint.tolerance_above = 0.05;
    wrist_2_constraint.tolerance_below = 0.05;
    wrist_2_constraint.weight = 0;

    moveit_msgs::msg::JointConstraint wrist_3_constraint;
    wrist_3_constraint.joint_name = "wrist_1_joint";
    wrist_3_constraint.position = -3 * M_PI / 4.0;
    wrist_3_constraint.tolerance_above = M_PI / 4.0;
    wrist_3_constraint.tolerance_below = M_PI / 4.0;
    wrist_3_constraint.weight = 0;

    moveit_msgs::msg::JointConstraint elbow_constraint;
    elbow_constraint.joint_name = "elbow_joint";
    elbow_constraint.position = M_PI / 2.0;
    elbow_constraint.tolerance_above = M_PI / 2.0;
    elbow_constraint.tolerance_below = 0.0;
    elbow_constraint.weight = 0;

    constraints.joint_constraints.push_back(wrist_2_constraint);
    constraints.joint_constraints.push_back(wrist_3_constraint);
    constraints.joint_constraints.push_back(elbow_constraint);
    move_group_interface->setPathConstraints(constraints);
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


void Planner::goalPoseCallback(const geometry_msgs::msg::PoseStamped &pose) {
    auto g_pose = pose;
    g_pose.pose.position.z += GRIPPER_HEIGHT + GRIPPER_OFFSET; // 
    goal_pose = g_pose;
}


bool Planner::isPoseClose(const geometry_msgs::msg::Pose &a,
                          const geometry_msgs::msg::Pose &b) {    
    return norm(a.position, b.position) < POSITION_PRECISION &&
           norm(a.orientation, b.orientation) < ORIENTATION_PRECISION;
}


inline double Planner::norm(const geometry_msgs::msg::Point &a,
                            const geometry_msgs::msg::Point &b) {
    tf2::Vector3 v1 = tf2::Vector3(a.x, a.y, a.z);
    tf2::Vector3 v2 = tf2::Vector3(b.x, b.y, b.z);
    return (v1 - v2).length();
}

inline double Planner::norm(const geometry_msgs::msg::Quaternion &a,
                            const geometry_msgs::msg::Quaternion &b) {
    tf2::Quaternion q1 = tf2::Quaternion(a.x, a.y, a.z, a.w);
    tf2::Quaternion q2 = tf2::Quaternion(b.x, b.y, b.z, b.w);
    return q1.angleShortestPath(q2);
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<Planner>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner);
    executor.spin();
    rclcpp::shutdown();
}