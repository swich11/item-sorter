import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from interfaces.msg import LabelledPoseArray, LabelledPose
from visualization_msgs.msg import Marker, MarkerArray

from enum import Enum

# Constant Parameters 
# TODO: make project variables
MIN_BIN_AREA_THRESHOLD = 2000 # TO ADJUST

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    RED1    = ((0, 70, 50), (10, 255, 255))
    RED2    = ((170, 70, 50), (180, 255, 255))
    GREEN   = ((35, 40, 40), (85, 255, 255))
    BLUE    = ((90, 50, 50), (140, 255, 255))
    YELLOW  = ((15, 100, 100), (35, 255, 255))

    @property
    def lower(self):
        return np.array(self.value[0], dtype=np.uint8)

    @property
    def upper(self):
        return np.array(self.value[1], dtype=np.uint8)

# Define object shapes
class ObjectShape(Enum):
    CYLINDER = 1
    SQUARE_PRISM = 2
    TRIANGULAR_PRISM = 3
    RECTANGULAR_PRISM = 4
    STAR_PRISM = 5
    HEXAGONAL_PRISM = 6

    UNKNOWN = 0

class objectDetect(Node):

    def __init__(self):
        super().__init__('object_detect')
        
        # depth camera subscriptions
        self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.colour_img_callback, 10)
        self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_img_callback, 10)
        self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback,10)
        self.intrinsics = None
        self.depth_image = None

        # Timer definitions
        self.routine_timer = self.create_timer(1, self.routine_callback)

        # Publishers
        self.object_pub = self.create_publisher(LabelledPoseArray, "/base/objects/labelled_pose_array", 10)
        self.goal_pub = self.create_publisher(LabelledPoseArray, "/base/objects/labelled_pose_array", 10) 
        self.marker_pub = self.create_publisher(MarkerArray, "/base/objects/markers", 10)
        
        # Transformation Interface
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # General Variables
        self.cv_image = None
        self.mask = None
        self.cv_bridge = CvBridge()

    def camera_info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    # This gets bgr image from the image topic and finds where green in the image is
    def colour_img_callback(self, msg):      
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error in colour_img_callback: {str(e)}")

    # This gets depth_frame aligned with RGB image
    def depth_img_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in depth_img_callback: {str(e)}")

    def pixel_to_global(self, pixel_pt):
        if self.depth_image is not None and self.intrinsics is not None:
            [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
            return [x, y, z]
        else:
            return None
    
    # TODO: Implement shape classification
    def classify_shape(self, contour):
        # Polygonal approximation
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True) # May need to find face among approximations
        vertices = len(approx)
        area = cv2.contourArea(contour)

        shape = ObjectShape.UNKNOWN
        if vertices == 3:
            shape = ObjectShape.TRIANGULAR_PRISM
        elif vertices == 4:
            shape = ObjectShape.SQUARE_PRISM
        elif 5 <= vertices <= 6:
            shape = ObjectShape.HEXAGONAL_PRISM
            
        is_bin = (area > MIN_BIN_AREA_THRESHOLD)
        
        return shape, is_bin
    
    def make_marker(self, idx, colour_range, position, is_bin):
        Marker_msg = Marker()
        Marker_msg.header.frame_id = "camera_color_optical_frame"
        Marker_msg.header.stamp = self.get_clock().now().to_msg()
        Marker_msg.ns = "detected_objects"
        Marker_msg.id = idx
        Marker_msg.type = Marker.SPHERE
        Marker_msg.action = Marker.ADD
        Marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
        Marker_msg.pose.orientation.w = 1.0
        Marker_msg.scale.x = 0.1 if is_bin else 0.05
        Marker_msg.scale.y = 0.1 if is_bin else 0.05
        Marker_msg.scale.z = 0.1 if is_bin else 0.05
        Marker_msg.color.a = 1.0
        colour_low = colour_range.value[0]
        colour_high = colour_range.value[1]
        Marker_msg.color.r = (colour_low[0]+colour_high[0])/(2*255)
        Marker_msg.color.g = (colour_low[1]+colour_high[1])/(2*255)
        Marker_msg.color.b = (colour_low[2]+colour_high[2])/(2*255)
        return Marker_msg
        
    def detect_objects(self, colour_img, depth_img):
        # Initialize msgs
        objects = LabelledPoseArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = "camera_color_optical_frame"
        goals = LabelledPoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = "camera_color_optical_frame"
        markers = MarkerArray()
        num_detected = 0
        
        if self.cv_image is None:
            return goals, objects, markers, None
        annotated = colour_img.copy()
        
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Define area thresholds # To be project parameters
        min_area = 500
        max_area = 50000
        
        # Loop through each colour range and detect objects of that colour
        for colour_range in ObjectColour:
            mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            # MIGHT NEED TO ADD MORPHOLOGICAL OPERATIONS HERE
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area or area > max_area:
                    continue
            
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    # Calculate the center of the object
                    cX = int(moments['m10'] / moments['m00'])
                    cY = int(moments['m01'] / moments['m00'])
                    
                    # Convert the pixel coordinates to 3D world coordinates
                    global_position = self.pixel_to_global([cX, cY])
                    if global_position is not None:
                        # Append the object to the list
                        shape, is_bin = self.classify_shape(contour)
                        num_detected += 1
                        if is_bin:
                            goal = LabelledPose()
                            goal.label = f"{colour_range.name}_{shape.name}_{num_detected}_goal"
                            goal.colour = colour_range.name
                            goal.shape = shape.name
                            goal.pose.position = Point(x=global_position[0], y=global_position[1], z=global_position[2])
                            goals.poses.append(goal)
                        else:
                            object = LabelledPose()
                            object.label = f"{colour_range.name}_{shape.name}_{num_detected}"
                            object.colour = colour_range.name
                            object.shape = shape.name
                            object.pose.position = Point(x=global_position[0], y=global_position[1], z=global_position[2])
                            objects.poses.append(object)
                            
                        # Create and append marker for visualization
                        Marker_msg = self.make_marker(num_detected, colour_range, global_position, is_bin)
                        markers.markers.append(Marker_msg)
                        
                    # Show the mask image
                    cv2.imshow("Mask", mask)
                    cv2.waitKey(1)  # Wait for a brief moment to update the window

                    # Draw the center on the original image for visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)

        # Sort detections by colour and shape for consistent ordering
        # detections.sort(key=lambda d: (d['colour'], d['shape']))
        return goals, objects, markers, annotated

    def broadcast_transform(self, object, idx):
        transform = TransformStamped()

        # Header info
        # idx for distinguishing multiple objects of same type
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "camera_frame"
        transform_stamped.child_frame_id = f'{object.colour}_{object.shape}_{idx}'

        # Set translation
        transform_stamped.transform.translation = Point(x=object.position[0], y=object.position[1], z=object.position[2])
        
        # Send the transform
        self.tf_broadcaster.sendTransform(transform_stamped)

    def routine_callback(self):
        if (self.cv_image is None):
            self.get_logger().info("No image received. Routine callback skipped.")
            return None

        goals, objects, markers, annotated = self.detect_objects(self.color_frame, self.depth_frame)

        # Publish detected objects
        self.object_pub.publish(objects)
        self.goal_pub.publish(goals)
        self.marker_pub.publish(markers)

        if annotated is not None:
            cv2.imshow('annotated', annotated)
            cv2.waitKey(1)
        return

def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()

if __name__ == '__main__':
    main()