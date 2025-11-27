#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/async_buffer_interface.hpp"
#include "tf2_ros/create_timer_interface.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <sstream>
#include <chrono>


#include "interfaces/srv/transform_lookup.hpp"



class TransformNode : public rclcpp::Node {
public:
  TransformNode();

  void broadcast_static_camera_transform();

  void lookup_service_callback(const std::shared_ptr<interfaces::srv::TransformLookup::Request> req,
                               const std::shared_ptr<interfaces::srv::TransformLookup::Response> res);

private:
    rclcpp::Service<interfaces::srv::TransformLookup>::SharedPtr lookup_service;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};
