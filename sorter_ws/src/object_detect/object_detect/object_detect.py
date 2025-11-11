import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
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
MIN_BIN_AREA_THRESHOLD = 1500 # TO ADJUST

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    # RED1    = ((0, 70, 50), (10, 255, 255))
    RED2    = ((170, 70, 50), (180, 255, 255))
    # GREEN   = ((35, 40, 40), (85, 255, 255))
    # BLUE    = ((100, 85, 85), (140, 255, 255))
    # YELLOW  = ((15, 100, 100), (35, 255, 255))

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
        self.depth_image = None
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

    # This gets depth_image aligned with RGB image
    def depth_img_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in depth_img_callback: {str(e)}")

    def pixel_to_global(self, pixel_pt):
        if self.depth_image is not None and self.intrinsics is not None and pixel_pt[0]<self.intrinsics.height and pixel_pt[1]<self.intrinsics.width:
            [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
            return [x, y, z]
        else:
            return None
    
    # TODO: Implement shape classification
    # Point cloud method to classify shape or 
    # ML based method could be implemented here
    def classify_shape(self, contour):
        if self.intrinsics is None:
            return ObjectShape.UNKNOWN, False, None
        
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        vertices = len(approx)
        area = cv2.contourArea(contour)

        shape = ObjectShape.CYLINDER
        zero_mask = np.zeros((self.intrinsics.height, self.intrinsics.width), dtype=np.uint8)
        mask = cv2.drawContours(zero_mask, [contour], -1, (0, 255, 0), -1)
        # shape, _, _ = self.fit_shape(self.depth_image, mask)
            
        # is_bin = self.is_bin_helper(contour, self.depth_image, mask)
        is_bin = cv2.contourArea(contour) > MIN_BIN_AREA_THRESHOLD
        
        return shape, is_bin, approx
    
    # Helper to determine if contour likely represents a bin
    # is vulnerable to occlusion and angle of view
    # good enough for before using point cloud method for more accurate info
    def is_bin_helper(self, contour, depth_img, mask):
        if self.intrinsics is None:
            return False
        
        # Get average depth inside the contour
        depth_values = depth_img[mask == 255]
        depth_values = depth_values[depth_values > 0]
        if len(depth_values) == 0:
            return False
        Z = np.median(depth_values) * 0.001  # convert mm → m

        # Get bounding box in pixels
        x, y, w, h = cv2.boundingRect(contour)

        # Convert pixel distance to meters at depth Z
        width_m = (w / self.intrinsics.fx) * Z
        height_m = (h / self.intrinsics.fy) * Z
        
        return (width_m > 0.06 or height_m > 0.06)
    
    # Convert masked depth image to an Open3D point cloud.
    def mask_to_pointcloud(self, depth_frame, mask):
        if self.intrinsics is None:
            return None
        
        indices = np.where(mask > 0)
        z = depth_frame[indices] / 1000.0  # mm → m
        u = indices[1]
        v = indices[0]
        x = (u - self.intrinsics.ppx) * z / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) * z / self.intrinsics.fy

        points = np.vstack((x, y, z)).T
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    # Estimate number of flat faces using convex hull.
    def count_hull_faces(self, pcd):
        try:
            hull, _ = pcd.compute_convex_hull()
            triangles = np.asarray(hull.triangles)
            vertices = np.asarray(hull.vertices)
            verts = np.asarray(hull.vertices)
            mesh = hull

            # Compute normals for each triangle
            triangle_normals = np.asarray(mesh.triangle_normals)

            # Cluster normals by angle similarity
            clusters = []
            threshold = np.deg2rad(15)  # within 15 degrees
            for n in triangle_normals:
                matched = False
                for c in clusters:
                    if np.arccos(np.clip(np.dot(n, c), -1.0, 1.0)) < threshold:
                        matched = True
                        break
                if not matched:
                    clusters.append(n)
            return len(clusters)
        except Exception:
            return 0

    ### Fit geometric model to classify prism shape.
    def fit_shape(self, depth_frame, mask):
        pcd = self.mask_to_pointcloud(depth_frame, mask)
        if len(pcd.points) < 100:
            return ObjectShape.UNKNOWN, None, 0

        # Clean noise
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

        # Get bounding box info
        bbox = pcd.get_oriented_bounding_box()
        extents = bbox.extent
        aspect = np.sort(extents) / np.max(extents)

        # Compute convex hull and count flat faces
        n_faces = self.count_hull_faces(pcd)

        # --- Infer shape based on faces + proportions ---
        if n_faces <= 0:
            shape = ObjectShape.UNKNOWN
        elif n_faces <= 6:
            shape = ObjectShape.TRIANGULAR_PRISM
        elif n_faces <= 8:
            shape = ObjectShape.SQUARE_PRISM
        elif n_faces <= 12:
            shape = ObjectShape.HEXAGONAL_PRISM
        else:
            shape = ObjectShape.UNKNOWN
            
        # Catch flat objects
        if aspect[0] < 0.2:
            shape = ObjectShape.UNKNOWN

        center = bbox.center
        return shape, center, n_faces
    
    def make_marker(self, idx, colour_range, position, is_bin):
        Marker_msg = Marker()
        Marker_msg.header.frame_id = "camera_frame"
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

        # colour_low = self.hsv_to_rgb(colour_range.value[0])
        # colour_high = self.hsv_to_rgb(colour_range.value[1])
        # Marker_msg.color.b = (colour_low[0]+colour_high[0])/(2*255)
        # Marker_msg.color.g = (colour_low[1]+colour_high[1])/(2*255)
        # Marker_msg.color.r = (colour_low[2]+colour_high[2])/(2*255)

        # Marker_msg.color.r = 255.0 if colour_range in [ObjectColour.RED1, ObjectColour.RED2, ObjectColour.YELLOW] else 0.0
        Marker_msg.color.r = 255.0 if colour_range in [ObjectColour.RED1, ObjectColour.RED2] else 0.0
        # Marker_msg.color.g = 255.0 if colour_range in [ObjectColour.GREEN, ObjectColour.YELLOW] else 0.0
        Marker_msg.color.g = 0.0
        Marker_msg.color.b = 255.0 if colour_range == ObjectColour.BLUE else 0.0
        return Marker_msg
    
    def make_goal(self, idx, colour_range, shape, position):
        goal = LabelledPose()
        goal.label = f"{colour_range.name}_{shape.name}_{idx}_goal"
        goal.colour = colour_range.name
        goal.shape = shape.name
        goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
        return goal
    
    def make_object(self, idx, colour_range, shape, position):
        object = LabelledPose()
        object.label = f"{colour_range.name}_{shape.name}_{idx}"
        object.colour = colour_range.name
        object.shape = shape.name
        object.pose.position = Point(x=position[0], y=position[1], z=position[2])
        return object
        
    def detect_objects(self, colour_img, depth_img):
        # Initialize msgs
        objects = LabelledPoseArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = "camera_frame"
        goals = LabelledPoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = "camera_frame"
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
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
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
                        shape, is_bin, _ = self.classify_shape(contour)
                        num_detected += 1
                        if is_bin:
                            goals.poses.append(self.make_goal(num_detected, colour_range, shape, global_position))
                        else:
                            objects.poses.append(self.make_object(num_detected, colour_range, shape, global_position))
                            
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

    # def hsv_to_rgb(self, hsv_color):
    #     hsv_color = np.array(hsv_color, dtype=np.float32) / np.array([180.0, 255.0, 255.0])
    #     rgb_color = cv2.cvtColor(np.uint8([[hsv_color]]), cv2.COLOR_HSV2RGB)[0][0]
    #     return rgb_color.astype(np.float32) / 255.0

    # For vision demo only
    def test(self):
        # Create some test markers for visualization
        markers = MarkerArray()
        test_positions = [
            [1.2, 0.4, -0.2],
            [1.2, 0.2, -0.2],
            [1.2, -0.1, -0.2],
            [1.2, -0.3, -0.2],
            [1.2, 0.0, -0.2]
        ]
        test_colours = [ObjectColour.RED1, ObjectColour.RED2, ObjectColour.RED1, ObjectColour.BLUE, ObjectColour.BLUE]
        
        for idx, pos in enumerate(test_positions):
            Marker_msg = self.make_marker(idx, test_colours[idx], pos, is_bin=(idx==0))
            markers.markers.append(Marker_msg)
            
        objects = LabelledPoseArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = "camera_frame"
        
        goals = LabelledPoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = "camera_frame"
        for idx, pos in enumerate(test_positions):
            if idx == 0:
                goal = LabelledPose()
                goal.label = f"{test_colours[idx].name}_{ObjectShape.CYLINDER.name}_{idx+1}_goal"
                goal.colour = test_colours[idx].name
                goal.shape = ObjectShape.CYLINDER.name
                goal.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
                goals.poses.append(goal)
                continue
            else:
                object = LabelledPose()
                object.label = f"{test_colours[idx].name}_{ObjectShape.CYLINDER.name}_{idx+1}"
                object.colour = test_colours[idx].name
                object.shape = ObjectShape.CYLINDER.name
                object.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
                objects.poses.append(object)

        goals.poses.append(objects.poses[0])  # First object as goal
        return goals, objects, markers

    def routine_callback(self):
        if (self.cv_image is None):
            self.get_logger().info("No image received. Routine callback skipped.")
            # return None

        goals, objects, markers, annotated = self.detect_objects(self.cv_image, self.depth_image)
        
        # For demo only without object detection
        # goals, objects, markers = self.test()
        # #

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