#include <map>
#include <string>
#include <vector>
#include <queue>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include "rclcpp/rclcpp.hpp"


#include "interfaces/msg/labelled_pose_array.hpp"
#include "interfaces/msg/labelled_pose.hpp"
#include "interfaces/srv/move.hpp"
#include "interfaces/srv/transform_lookup_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/empty.hpp"


#include "brain/Semaphore.hpp"


constexpr int MOVING_AVERAGE_LEN = 5;


class ObjectAveragedPose {
public:
    ObjectAveragedPose() {
        Eigen::Vector3d zeroed_point(0.0, 0.0, 0.0);
        prev_points.fill(Eigen::Vector3d(0.0, 0.0, 0.0));
        prev_orients.fill(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));
        i = 0; // circular buffer index
        elements_filled = 0;
    }


    void update_pose(const geometry_msgs::msg::PoseStamped &pose) {
        // Check difference between timestamps < 0.5s
        if (rclcpp::Time(pose.header.stamp) - rclcpp::Time(last_header.stamp) 
                > rclcpp::Duration(0, 500000000)) {
            // flush moving average -> confidence goes to 0.0
            elements_filled = 0;
            prev_points.fill(Eigen::Vector3d(0.0, 0.0, 0.0));
            prev_orients.fill(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));
        }
        last_header = pose.header;

        Eigen::Vector3d point(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        );
        Eigen::Vector4d quat(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        );
        prev_points[i] = point;
        prev_orients[i] = quat;
        i++;
        if (elements_filled < MOVING_AVERAGE_LEN) {
            elements_filled ++;
        }
        if (i == MOVING_AVERAGE_LEN) {
            i = 0;
        }
    }

    
    double calculate_position_conf() {
        if (elements_filled < MOVING_AVERAGE_LEN) {
            return 0.0;
        }
        Eigen::Vector3d mean = std::accumulate(prev_points.begin(), prev_points.end(), Eigen::Vector3d(0.0, 0.0, 0.0))
                                / static_cast<float>(MOVING_AVERAGE_LEN);
        Eigen::Vector3d std(0.0, 0.0, 0.0);
        std::for_each(prev_points.begin(), prev_points.end(), 
            [&std, mean](const Eigen::Vector3d &point) {
                std[0] += (point[0] - mean[0]) / static_cast<float>(MOVING_AVERAGE_LEN);
                std[1] += (point[1] - mean[1]) / static_cast<float>(MOVING_AVERAGE_LEN);
                std[2] += (point[2] - mean[2]) / static_cast<float>(MOVING_AVERAGE_LEN);
            }
        );
        auto normalised_var = std.norm() / mean.norm();
        normalised_var *= normalised_var;
        return 1 / (1 + normalised_var); // confidence score
    }


    double calculate_orientation_conf() {
        if (elements_filled < MOVING_AVERAGE_LEN) {
            return 0.0;
        }
        Eigen::Vector4d mean = std::accumulate(prev_orients.begin(), prev_orients.end(), Eigen::Vector4d(0.0, 0.0, 0.0, 0.0))
                                    / static_cast<float>(MOVING_AVERAGE_LEN);
        Eigen::Vector4d std(0.0, 0.0, 0.0, 0.0);
        std::for_each(prev_orients.begin(), prev_orients.end(),
            [&std, mean](const Eigen::Vector4d &quat) {
                std[0] += (quat[0] - mean[0]) / static_cast<float>(MOVING_AVERAGE_LEN);
                std[1] += (quat[1] - mean[1]) / static_cast<float>(MOVING_AVERAGE_LEN);
                std[2] += (quat[2] - mean[2]) / static_cast<float>(MOVING_AVERAGE_LEN);
                std[3] += (quat[3] - mean[3]) / static_cast<float>(MOVING_AVERAGE_LEN);
            }
        );
        auto normalised_var = std.norm() / mean.norm();
        normalised_var *= normalised_var;
        return 1 / (1 + normalised_var); // confidence score
    }


    double calculate_conf() {
        return std::min(calculate_position_conf(), calculate_orientation_conf());
    }


    geometry_msgs::msg::PoseStamped get_pose() {
        geometry_msgs::msg::PoseStamped ret;
        ret.header = last_header;

        Eigen::Vector4d mean_quat = std::accumulate(prev_orients.begin(), prev_orients.end(), Eigen::Vector4d(0.0, 0.0, 0.0, 0.0))
                                    / static_cast<float>(MOVING_AVERAGE_LEN);
        Eigen::Vector3d mean_pos = std::accumulate(prev_orients.begin(), prev_orients.end(), Eigen::Vector3d(0.0, 0.0, 0.0))
                                    / static_cast<float>(MOVING_AVERAGE_LEN);
        ret.pose.position.x = mean_pos.x();
        ret.pose.position.y = mean_pos.y();
        ret.pose.position.z = mean_pos.z();

        ret.pose.orientation.w = mean_quat.w();
        ret.pose.orientation.x = mean_quat.x();
        ret.pose.orientation.y = mean_quat.y();
        ret.pose.orientation.z = mean_quat.z();

        return ret;
    }

private:
    int i; // index for circular buffer;
    int elements_filled;

    std::array<Eigen::Vector3d, MOVING_AVERAGE_LEN> prev_points;
    std::array<Eigen::Vector4d, MOVING_AVERAGE_LEN> prev_orients;

    std_msgs::msg::Header last_header;
};


struct ItemPose {
    ObjectAveragedPose pose;
    bool in_queue;
};


class Brain : public rclcpp::Node {
public:
    Brain();

    ~Brain();

    // Debugging function
    void send_move_request(const geometry_msgs::msg::PoseStamped &pose);

    void object_topic_callback(const interfaces::msg::LabelledPoseArray &msg);

    void send_move_request(const std::string &label);

    bool move_request_response(rclcpp::Client<interfaces::srv::Move>::SharedFuture future);

    inline void publish_goal_pose(const std::string &label);

    inline void publish_item_pose(const std::string &label);

    std::string get_goal_label(const std::string &item_label);

    void transform_labelled_pose_array(const interfaces::msg::LabelledPoseArray &msg,
                                       std::function<void(const interfaces::msg::LabelledPoseArray&)> f_update_map);

    void update_item_map(const interfaces::msg::LabelledPoseArray &msg);

    void update_goal_map(const interfaces::msg::LabelledPoseArray &msg);

    void spin_wait_for_items();

private:
    bool running;
    Semaphore item_queue_sem{0};
    std::thread move_call_thread;
    std::queue<std::string> item_queue;
    std::map<std::string, ItemPose> item_pose_map;
    std::map<std::string, ObjectAveragedPose> goal_pose_map;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<interfaces::srv::Move>::SharedPtr move_client;
    rclcpp::Client<interfaces::srv::TransformLookupArray>::SharedPtr transform_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_update_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_move_publisher;
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr item_pose_subscription;
};