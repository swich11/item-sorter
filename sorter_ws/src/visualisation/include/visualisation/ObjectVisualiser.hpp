#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "interfaces/msg/labelled_pose.hpp"
#include "interfaces/msg/labelled_pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"

enum ObjectID {
    RedSquare,
    RedCircle,
    RedHexagon,
    BlueSquare,
    BlueCircle,
    BlueHexagon,
    GreenSquare,
    GreenCircle,
    GreenHexagon,
    CircleBucket,
    SquareBucket,
    HexagonBucket,
};


struct ObjectMarkerInfo {
    ObjectID id;
    int type;
    std_msgs::msg::ColorRGBA color;
    geometry_msgs::msg::Vector3 scale;
    bool has_mesh;
    std::string mesh_resource;

    ObjectMarkerInfo(ObjectID id, int type, float r, 
                     float g, float b, float x_scale,
                     float y_scale, float z_scale, bool has_mesh,
                     std::string mesh_resource = "") {
        this->id = id;
        this->type = type;
        this->color.r = r;
        this->color.g = g;
        this->color.b = b;
        this->color.a = 1.0; // assume non-transparent
        
        this->scale.x = x_scale;
        this->scale.y = y_scale;
        this->scale.z = z_scale;

        this->has_mesh = has_mesh;
        this->mesh_resource = mesh_resource;
    }
};


class ObjectVisualiser : public rclcpp::Node {
public:
    ObjectVisualiser();

    void object_array_callback(const interfaces::msg::LabelledPoseArray &msg);

private:
    rclcpp::Subscription<interfaces::msg::LabelledPoseArray>::SharedPtr object_pose_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    std::map<std::string, ObjectMarkerInfo> marker_label_map;
    visualization_msgs::msg::MarkerArray last_marker_array;
    static constexpr float object_size = 0.04;
    static constexpr float bin_size = 0.1;
    static constexpr float mesh_size = 1.0;
};