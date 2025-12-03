#include "moveit_planner/Planner.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


Planner::Planner() : Node("planner") {}


void Planner::initialiseMoveIt() {
    auto self = shared_from_this();
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(self, "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);
    move_group_interface->allowReplanning(true);
    move_group_interface->startStateMonitor(3.0);


    planning_scene_monitor = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(self, "robot_description");


    std::string frame_id = move_group_interface->getPlanningFrame();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {
        generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"),
        generateCollisionObject(0.04, 1.2, 1.0, -0.20, 0.25, 0.5, frame_id, "sideWall"),
        generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.02, frame_id, "table"),
    };
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    // Setup allowed collisions
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor);
    auto& acm = planning_scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry("table", "base_link_inertia", true);

    this->received_joint_states = false;

    grabbed_home_pose = false;
    move_canceled = false;

    planner_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    goal_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/brain/move/pose", 10, std::bind(&Planner::goalPoseCallback, this, _1));
    cancel_move_subscription = this->create_subscription<std_msgs::msg::Empty>("/brain/move/cancel", 10, std::bind(&Planner::cancelMoveCallback, this, _1));
    joint_state_subscription = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&Planner::jointStateCallback, this, _1));
    move_server = this->create_service<interfaces::srv::Move>("/moveit_planner/move", std::bind(&Planner::moveServiceCallback, this, _1, _2), 
                                                                rmw_qos_profile_services_default, planner_callback_group);
    arduino_pub = this->create_publisher<std_msgs::msg::String>("/arduino_cmds", 10);   // initialize publisher to send commands to Arduino
    RCLCPP_INFO(this->get_logger(), "Planner Launched. Ready for Commands");
}


void Planner::moveServiceCallback(std::shared_ptr<interfaces::srv::Move::Request> req,
                                  std::shared_ptr<interfaces::srv::Move::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Move Request.");
    if (!grabbed_home_pose) {
        // Initialise the home pose
        home_pose = move_group_interface->getCurrentPose().pose;
        home_pose.orientation.x = sqrt(2) / 2.0;
        home_pose.orientation.y = -sqrt(2) / 2.0;
        home_pose.orientation.z = 0.0;
        home_pose.orientation.w = 0.0;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, w: %f", home_pose.orientation.x, 
                                                                      home_pose.orientation.y,
                                                                      home_pose.orientation.z,
                                                                      home_pose.orientation.w);
        grabbed_home_pose = true;
        
        // For now unless we add pose rotation
        goal_pose.pose.orientation = home_pose.orientation;
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.header.frame_id = "base_link";
    }

    if (this->received_joint_states) {
        move_group_interface->stop();
        // Sets the path constraints for third wrist to avoid revolutions.
        move_group_interface->setPathConstraints(getGripPathConstraints());
        move(res, req->grasp, req->pose);
    } else {
        res->message = "Have not received joint states yet.";
        res->success = false;
    }
}


bool Planner::move(std::shared_ptr<interfaces::srv::Move::Response> res, bool grasp, geometry_msgs::msg::Pose goal) {
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f, w: %f", goal_pose.pose.orientation.x, 
    //                                                               goal_pose.pose.orientation.y,
    //                                                               goal_pose.pose.orientation.z,
    //                                                               goal_pose.pose.orientation.w);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::msg::Pose tracked_goal = home_pose; // Anything different to the goal_pose works
    goal_pose.pose = goal;
    goal_pose.pose.orientation = home_pose.orientation;
    goal_pose.pose.position.z += GRIPPER_HEIGHT + GRIPPER_OFFSET;

    do {
        if (move_canceled) {
            // Move canceled by brain, don't continue
            move_group_interface->stop();
            res->message = "Planning was cancelled during move.";
            res->success = false;
            return false;
        }
        if (!isPoseClose(tracked_goal, goal_pose.pose)) {
            // Change goal when the object moves
            tracked_goal = goal_pose.pose;
            RCLCPP_INFO(this->get_logger(), "Tracked goal: x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                   tracked_goal.position.y,
                                                                   tracked_goal.position.z);
            move_group_interface->setPoseTarget(tracked_goal);
            move_group_interface->stop();
            auto ret = move_group_interface->plan(plan);
            if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Planning Failed :(.");
                res->success = false;
                res->message = "Planning failed during move: " + moveit::core::error_code_to_string(ret);
                return false;
            }
            move_group_interface->asyncExecute(plan);
        }
        // rclcpp::sleep_for(50ms);
    } while (!move_group_interface->getMoveGroupClient().action_server_is_ready() || !isPoseClose(tracked_goal, goal_pose.pose));
    // Pose is close to tracked goal.
    RCLCPP_INFO(this->get_logger(), "Completed move above goal pose :)");
    // drop from height
    if (!grasp) {
        RCLCPP_INFO(this->get_logger(), "Dropping");
        this->ungrasp();
        res->success = true;
        return true;
    }
    
    // Move downwards for grasp
    auto grab_pose = goal_pose.pose;
    grab_pose.orientation = home_pose.orientation;
    grab_pose.position.z += GRAB_OFFSET - GRIPPER_OFFSET;
    move_group_interface->setPoseTarget(grab_pose);
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
    RCLCPP_INFO(this->get_logger(), "Moving to grab.");
    ret = move_group_interface->execute(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "Execute Failed :(.");
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                tracked_goal.position.y,
                                                                tracked_goal.position.z);
        res->message = "Execute failed during grab: " + moveit::core::error_code_to_string(ret);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Grasping.");
    this->grasp();

    // Move back up after grasp
    grab_pose.position.z += GRIPPER_OFFSET - GRAB_OFFSET;
    move_group_interface->setPoseTarget(grab_pose);
    move_group_interface->stop();
    ret = move_group_interface->plan(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Planning Failed :(.");
        res->success = false;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                tracked_goal.position.y,
                                                                tracked_goal.position.z);
        res->message = "Planning failed during move up from grab: " + moveit::core::error_code_to_string(ret);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Moving to grab.");
    ret = move_group_interface->execute(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "Execute Failed :(.");
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", tracked_goal.position.x,
                                                                tracked_goal.position.y,
                                                                tracked_goal.position.z);
        res->message = "Execute failed during move up from grab: " + moveit::core::error_code_to_string(ret);
        return false;
    }
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
    rclcpp::sleep_for(1s);
}

// sends serial msg "open" over port to teensy
void Planner::ungrasp() {
    std_msgs::msg::String msg;
    msg.data = "open\n";
    arduino_pub->publish(msg);
    rclcpp::sleep_for(1s);
}

moveit_msgs::msg::Constraints Planner::getPathConstraints() {
    // Lock wrist 2 link to simplify path planning
    moveit_msgs::msg::Constraints constraints;
    try {
        moveit_msgs::msg::JointConstraint wrist_2_constraint;
        wrist_2_constraint.joint_name = "wrist_2_joint";
        wrist_2_constraint.position = last_joint_values.at(wrist_2_constraint.joint_name);
        wrist_2_constraint.tolerance_above = M_PI / 4.0;
        wrist_2_constraint.tolerance_below = M_PI / 4.0;
        wrist_2_constraint.weight = 1.0;

        moveit_msgs::msg::JointConstraint wrist_1_constraint;
        wrist_1_constraint.joint_name = "wrist_1_joint";
        wrist_1_constraint.position = last_joint_values.at(wrist_1_constraint.joint_name);
        wrist_1_constraint.tolerance_above = 15 * M_PI / 180.0;
        wrist_1_constraint.tolerance_below = 75 * M_PI / 180.0;
        wrist_1_constraint.weight = 1.0;

        moveit_msgs::msg::JointConstraint elbow_constraint;
        elbow_constraint.joint_name = "elbow_joint";
        elbow_constraint.position = last_joint_values.at(elbow_constraint.joint_name);
        elbow_constraint.tolerance_above = M_PI / 2.0;
        elbow_constraint.tolerance_below = 0.2;
        elbow_constraint.weight = 1.0;

        constraints.joint_constraints.push_back(wrist_2_constraint);
        constraints.joint_constraints.push_back(wrist_1_constraint);
        constraints.joint_constraints.push_back(elbow_constraint);
    } catch (std::out_of_range&) {
        RCLCPP_ERROR(this->get_logger(), "Tried to read from unavailable joint states.");
    }
    return constraints;
}


moveit_msgs::msg::Constraints Planner::getGripPathConstraints() {
    auto constraints = this->regular_constraints;
    moveit_msgs::msg::JointConstraint wrist_3_constraint;
    try {
        wrist_3_constraint.joint_name = "wrist_3_joint";
        wrist_3_constraint.position = last_joint_values.at(wrist_3_constraint.joint_name); // wrist_3_joint index
        // Avoid joint limits
        if (wrist_3_constraint.position > M_PI) {
            double diff = 2*M_PI - wrist_3_constraint.position;
            wrist_3_constraint.tolerance_above = diff - 0.1;
            wrist_3_constraint.tolerance_below = 2*M_PI - diff + 0.1;
        } else if (wrist_3_constraint.position < -M_PI) {
            double diff = -2*M_PI - wrist_3_constraint.position;
            wrist_3_constraint.tolerance_above = 2*M_PI - diff + 0.1;
            wrist_3_constraint.tolerance_below = diff - 0.1;
        } else {
            wrist_3_constraint.tolerance_above = M_PI;
            wrist_3_constraint.tolerance_below = M_PI;
        }
        wrist_3_constraint.weight = 1.0;
    } catch (std::out_of_range&) {
        RCLCPP_ERROR(this->get_logger(), "Tried to read from unavailable joint states.");
    }
    constraints.joint_constraints.push_back(wrist_3_constraint);
    return constraints;
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
    if (!grabbed_home_pose) {
        return;
    }
    auto g_pose = pose;
    g_pose.pose.orientation = home_pose.orientation;
    g_pose.pose.position.z += GRIPPER_HEIGHT + GRIPPER_OFFSET;
    goal_pose = g_pose;
}


void Planner::cancelMoveCallback(const std_msgs::msg::Empty&) {
    // Cancel move call at some point
}


void Planner::jointStateCallback(const sensor_msgs::msg::JointState &joint_state) {
    int i = 0;
    for (auto &name : joint_state.name) {
        last_joint_values[name] = joint_state.position[i];
        ++i;
    }
    if (!this->received_joint_states) {
        this->regular_constraints = getPathConstraints();
        // grip constraints must be called after regular constraints
        move_group_interface->setPathConstraints(regular_constraints);
        this->received_joint_states = true;
    }
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
    planner->initialiseMoveIt();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner);
    executor.spin();
    rclcpp::shutdown();
}