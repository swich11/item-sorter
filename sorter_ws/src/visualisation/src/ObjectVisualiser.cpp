#include "visualisation/ObjectVisualiser.hpp"


using std::placeholders::_1;


ObjectVisualiser::ObjectVisualiser() : Node("visualiser") {
    object_pose_sub = this->create_subscription<interfaces::msg::LabelledPoseArray>(
        "/camera/objects/labelled_pose_array",
        10,
        std::bind(&ObjectVisualiser::object_array_callback, this, _1)
    );
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualisation/object_array", 10
    );
    using namespace visualization_msgs::msg;
    marker_label_map = {
        {"RedSquare", ObjectMarkerInfo{ObjectID::RedSquare, Marker::CUBE, 255.0, 0.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"RedCircle", ObjectMarkerInfo{ObjectID::RedCircle, Marker::SPHERE, 255.0, 0.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"RedHexagon", ObjectMarkerInfo{ObjectID::RedHexagon, Marker::CYLINDER, 255.0, 0.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"GreenSquare", ObjectMarkerInfo{ObjectID::GreenSquare, Marker::CUBE, 0.0, 255.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"GreenCircle", ObjectMarkerInfo{ObjectID::GreenCircle, Marker::SPHERE, 0.0, 255.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"GreenHexagon", ObjectMarkerInfo{ObjectID::GreenHexagon, Marker::CYLINDER, 0.0, 255.0, 0.0, 0.05, 0.05, 0.05, false}},
        {"BlueSquare", ObjectMarkerInfo{ObjectID::BlueSquare, Marker::CUBE, 0.0, 0.0, 255.0, 0.05, 0.05, 0.05, false}},
        {"BlueCircle", ObjectMarkerInfo{ObjectID::BlueCircle, Marker::SPHERE, 0.0, 0.0, 255.0, 0.05, 0.05, 0.05, false}},
        {"BlueHexagon", ObjectMarkerInfo{ObjectID::BlueHexagon, Marker::CYLINDER, 0.0, 0.0, 255.0, 0.05, 0.05, 0.05, false}},
        {"CircleBucket", ObjectMarkerInfo{ObjectID::CircleBucket, Marker::CYLINDER, 255.0, 0.0, 0.0, 0.1, 0.1, 0.1, false}},
        {"SquareBucket", ObjectMarkerInfo{ObjectID::SquareBucket, Marker::CYLINDER, 0.0, 0.0, 255.0, 0.1, 0.1, 0.1, false}},
        {"HexagonBucket", ObjectMarkerInfo{ObjectID::HexagonBucket, Marker::CYLINDER, 0.0, 255.0, 0.0, 0.1, 0.1, 0.1, false}},
    };
}


void ObjectVisualiser::object_array_callback(const interfaces::msg::LabelledPoseArray &msg) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    try {
        for (auto &l_pose : msg.poses) {
            auto marker = visualization_msgs::msg::Marker();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = l_pose.pose;
            marker.ns = "object_marker";
            marker.header = msg.header;
            
            auto object_info = marker_label_map.at(l_pose.label);
            marker.id = object_info.id;
            marker.type = object_info.type;
            marker.scale = object_info.scale;
            marker.color = object_info.color;
            marker_array.markers.push_back(marker);
        }
    } catch (std::out_of_range &e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return;
    }
    marker_pub->publish(marker_array);
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectVisualiser>());
    rclcpp::shutdown();
}