import rclpy
import cv2
import cv2.aruco as aruco
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
MIN_BIN_AREA_THRESHOLD = 1500 # TO ADJUST
IS_TEST = True  # Set to True to enable test mode
MARKER_SIZE = 0.025  # Marker size in meters

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    RED    = ((0, 100, 75), (10, 255, 255))
    RED2    = ((170, 100, 75), (180, 255, 255))
    GREEN   = ((35, 100, 50), (85, 255, 255))
    BLUE    = ((100, 200, 30), (110, 255, 255))
    YELLOW  = ((15, 100, 50), (35, 255, 255))

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
    SPHERE = 7
    CUBE = 8

    UNKNOWN = 0
    
    @property
    def marker_type(self):
        if self == ObjectShape.CYLINDER:
            return Marker.CYLINDER
        elif self in [ObjectShape.SQUARE_PRISM, ObjectShape.RECTANGULAR_PRISM, ObjectShape.CUBE]:
            return Marker.CUBE
        elif self == ObjectShape.SPHERE:
            return Marker.SPHERE
        else:
            return Marker.SPHERE  # Default marker type
        
object_info = {
    0: {"shape" : ObjectShape.UNKNOWN, "is_bin" : False, "tf_to_centre" : (0,0,-0.02), "marker_size": 0.025},
    1: {"shape" : ObjectShape.SPHERE, "is_bin" : False, "tf_to_centre" : (0,0,-0.02), "marker_size": 0.025},
    2: {"shape" : ObjectShape.CUBE, "is_bin" : False, "tf_to_centre" : (0,0, -0.02) , "marker_size": 0.025},
    3: {"shape" : ObjectShape.UNKNOWN, "is_bin" : False, "tf_to_centre" : (0,0,-0.02) , "marker_size": 0.025},
    4: {"shape" : ObjectShape.CYLINDER, "is_bin" : True, "tf_to_centre" : (0,0,-0.04)   , "marker_size": 0.04},
    5: {"shape" : ObjectShape.RECTANGULAR_PRISM, "is_bin" : True, "tf_to_centre" : (0,0,-0.04) , "marker_size": 0.04},
    # ... add more as needed
    49: {"shape" : ObjectShape.UNKNOWN, "is_bin" : True, "tf_to_centre" : (0,0,-0.04) , "marker_size": 0.04},
}

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
        self.object_pub = self.create_publisher(LabelledPoseArray, "/camera/objects/labelled_pose_array", 10)
        self.goal_pub = self.create_publisher(LabelledPoseArray, "/camera/goals/labelled_pose_array", 10) 
        self.marker_pub = self.create_publisher(MarkerArray, "/camera/markers", 10)
        
        # Initialize Aruco parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()
        self.camera_matrix = None # updated with intrinsics
        self.dist_coeffs = None
        
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
            
            self.camera_matrix = np.array([
                [self.intrinsics.fx,  0, self.intrinsics.ppx],
                [0,  self.intrinsics.fy, self.intrinsics.ppy],
                [0,  0,   1]
            ])
            self.dist_coeffs = np.array(self.intrinsics.coeffs)
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

    # TODO : Check axis
    def pixel_to_global(self, pixel_pt, depth_offset=0.0):
        cX, cY = pixel_pt
        if self.depth_image is not None and self.intrinsics is not None and cX < self.intrinsics.width and cY < self.intrinsics.height:
            [z,y,x] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (cX, cY), self.depth_image[cY,cX]*0.001 + depth_offset)
            return [x, y, z]
        else:
            return None
    
    # TODO: Implement shape classification
    def classify_shape(self, contour):
        if self.intrinsics is None:
            return ObjectShape.UNKNOWN, False, None
        
        # Approximate the contour to reduce number of points
        # and clean up for visualization
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        
        shape = ObjectShape.UNKNOWN  # Default shape
        
        area = cv2.contourArea(contour)
        is_bin = area > MIN_BIN_AREA_THRESHOLD
        # zero_mask = np.zeros((self.intrinsics.height, self.intrinsics.width), dtype=np.uint8)
        # mask = cv2.drawContours(zero_mask, [contour], -1, (0, 255, 0), -1)
        # is_bin = self.is_bin_helper(contour, self.depth_image, mask)
        
        return shape, is_bin, approx
    
    # Helper to determine if contour likely represents a bin
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
        
    def make_marker(self, idx, colour_range, position, is_bin, shape=None, orientation=None):
        Marker_msg = Marker()
        Marker_msg.header.frame_id = "camera_link"
        Marker_msg.header.stamp = self.get_clock().now().to_msg()
        Marker_msg.ns = "detected_objects"
        Marker_msg.id = idx
        Marker_msg.type = shape.marker_type if shape is not None else (Marker.CYLINDER if is_bin else Marker.SPHERE)
        Marker_msg.action = Marker.ADD
        
        Marker_msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
        if orientation is not None:
            Marker_msg.pose.orientation = orientation
        else:
            Marker_msg.pose.orientation.w = 1.0
            Marker_msg.pose.orientation.x = 0.0
            Marker_msg.pose.orientation.y = 0.0
            Marker_msg.pose.orientation.z = 0.0
            
        Marker_msg.scale.x = 0.1 if is_bin else 0.05
        Marker_msg.scale.y = 0.1 if is_bin else 0.05
        Marker_msg.scale.z = 0.1 if is_bin else 0.05
        Marker_msg.color.a = 1.0

        Marker_msg.color.r = 255.0 if colour_range in [ObjectColour.RED, ObjectColour.RED2, ObjectColour.YELLOW] else 0.0
        Marker_msg.color.g = 255.0 if colour_range in [ObjectColour.GREEN, ObjectColour.YELLOW] else 0.0
        Marker_msg.color.b = 255.0 if colour_range == ObjectColour.BLUE else 0.0
        return Marker_msg
    
    def make_goal(self, idx, colour_range, shape, position, orientation=None):
        goal = LabelledPose()
        goal.label = f"{colour_range.name}_{shape.name}_{idx}_goal"
        goal.colour = colour_range.name
        goal.shape = shape.name
        
        goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
        if orientation is not None:
            goal.pose.orientation = orientation
        else:
            goal.pose.orientation.w = 1.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
        return goal
    
    def make_object(self, idx, colour_range, shape, position, orientation=None):
        object = LabelledPose()
        object.label = f"{colour_range.name}_{shape.name}_{idx}"
        object.colour = colour_range.name
        object.shape = shape.name
        
        object.pose.position = Point(x=position[0], y=position[1], z=position[2])
        if orientation is not None:
            object.pose.orientation = orientation
        else:
            object.pose.orientation.w = 1.0
            object.pose.orientation.x = 0.0
            object.pose.orientation.y = 0.0
            object.pose.orientation.z = 0.0
        return object
    
    def get_colour_masks(self):
        if self.cv_image is None:
            return None, None
        masks = {}
        complete_mask = np.zeros(self.cv_image.shape[:2], dtype=np.uint8)
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        for colour_range in ObjectColour:
            if colour_range == ObjectColour.RED2:
                continue
            # Create a mask for the current colour range
            mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            if colour_range == ObjectColour.RED:
                # Combine masks for RED and RED2
                mask2 = cv2.inRange(hsv_image, ObjectColour.RED2.lower, ObjectColour.RED2.upper)
                mask = cv2.bitwise_or(mask, mask2)
            
            # Apply morphological operations to clean up the mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=4)
            mask = cv2.GaussianBlur(mask, (9, 9), 0)
            
            masks[colour_range] = mask
            complete_mask = cv2.bitwise_or(complete_mask, mask)
        
        return masks, complete_mask
    
    def get_colour_contours(self, masks):
        all_contours = []
        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500 and area < 50000:
                    all_contours.append((colour, contour))
        return all_contours
    
    def find_aruco(self, contour = None):
        if self.colour_image is None or self.camera_matrix is None or self.dist_coeffs is None or self.intrinsics is None:
            return None, None, None, None
        
        x,y = 0,0
        tol = int(self.intrinsics.width // 10)  # 10% tolerance
        # make ROI around contour for aruco detection
        if contour is not None:
            x, y, w_box, h_box = cv2.boundingRect(contour)
            roi = self.colour_image[max(y-tol,0):min(y+h_box+tol,self.colour_image.shape[0]), max(x-tol,0):min(x+w_box+tol,self.colour_image.shape[1])]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)         
        else:
            gray = cv2.cvtColor(self.colour_image, cv2.COLOR_BGR2GRAY)
    
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # Adjust corners from ROI coordinates to full image coordinates
        if contour is not None:
            for pts in corners:
                pts += np.array([[max(x-tol,0), max(y-tol,0)]])
        
        tvecs, rvecs = [], []
        centers = []
        if ids is not None:
            for i in range(len(ids)):
                pts = corners[i][0]
                Cx = int(pts[:,0].mean())
                Cy = int(pts[:,1].mean())
                centers.append((Cx, Cy))
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.get_aruco_size(ids[i]), self.camera_matrix, self.dist_coeffs
                )
                rvecs.append(rvec)
                tvecs.append(tvec)
        
        return ids, tvecs, rvecs, centers
    
    def get_aruco_size(self, id):
        if id is None:
            return MARKER_SIZE
        
        id_int = int(id)
        if id_int in object_info:
            info = object_info[id_int]
            return info["marker_size"]
        else:
            return MARKER_SIZE
    
    def get_aruco_info(self, id):
        if id is None:
            return ObjectShape.UNKNOWN, False, (0,0,0), MARKER_SIZE
        
        id_int = int(id)
        if id_int in object_info:
            info = object_info[id_int]
            return info["shape"], info["is_bin"], info["tf_to_centre"], info["marker_size"]
        else:
            return ObjectShape.UNKNOWN, False, (0,0,0), MARKER_SIZE
        
    def point_transform(self, point, orientation, transform):
        R, _ = cv2.Rodrigues(orientation)
        t = np.array(transform).reshape((3,1))
        transform = np.array(point).reshape((3,1))
        return R @ transform + t
    
    def euler_distance(self, euler1, euler2):
        return np.sqrt((euler1[0]-euler2[0])**2 + (euler1[1]-euler2[1])**2 + (euler1[2]-euler2[2])**2)
    
    def get_contour_center(self, contour):
        moments = cv2.moments(contour)
        is_valid = moments['m00'] != 0
        if is_valid:
            cX = int(moments['m10'] / moments['m00'])
            cY = int(moments['m01'] / moments['m00'])
            return cX, cY, is_valid
        else:
            return 0, 0, is_valid

    def detect_objects(self):
        # Initialize msgs
        objects = LabelledPoseArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = "camera_link"
        goals = LabelledPoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = "camera_link"
        markers = MarkerArray()
        
        if self.cv_image is None:
            return goals, objects, markers, None, None
        
        annotated = self.cv_image.copy()
        num_detected = 0
        
        # Get masks for each colour range
        masks, complete_mask = self.get_colour_masks()
        
        # Get all contours from all colour masks
        contours = self.get_colour_contours(masks)

        # Loop through each contour to classify and locate objects
        for colour_range, contour in contours:
            # detect aruco marker 
            ids, tvecs, rvecs, centers = self.find_aruco(contour)
            
            if ids is not None and tvecs is not None and rvecs is not None and centers is not None:
                # ArUco detected, use it to classify shape
                detected = []
                
                for idx, id in enumerate(ids):
                    shape, is_bin, transform, _ = self.get_aruco_info(id)
                    
                    cX, cY = centers[idx]
                                
                    num_detected += 1        
                    global_position = self.pixel_to_global([cX, cY])
                    
                    # Refine global position using contour center to prevent skewed detections
                    moment_cX, moment_cY, is_valid = self.get_contour_center(contour)
                    if is_valid:
                    # if False:
                        tx,ty,tz = transform
                        R, _ = cv2.Rodrigues(rvecs[idx][0])
                        SwapXZ = np.array( [[0,0,1],
                                            [0,1,0],
                                            [1,0,0]])
                        rvec_flippedXZ, _ = cv2.Rodrigues(R @ SwapXZ)
                        contour_global = self.pixel_to_global([moment_cX, moment_cY], depth_offset=0.04 if is_bin else 0.02)
                        global_position0 = self.point_transform(global_position, rvecs[idx][0], (tx,ty,tz))
                        global_position1 = self.point_transform(global_position, rvec_flippedXZ, (tx,ty,-tz))
                        global_position2 = self.point_transform(global_position, rvecs[idx][0], (tx,ty,tz))
                        global_position3 = self.point_transform(global_position, rvec_flippedXZ, (tx,ty,-tz))
                        global_positions = [global_position0, global_position1, global_position2, global_position3]
                        dists = [np.linalg.norm(np.array(contour_global)-np.array(gp)) for gp in global_positions]
                        global_position = global_positions[dists.index(min(dists))]
                    else:
                        # as was before
                        global_position = self.point_transform(global_position, rvecs[idx][0], transform)
                    
                    # convert rvec to orientation quaternion
                    # orientation = self.euler_to_quaternion(rvecs[idx][0])
                    detected.append({
                        'id': num_detected,
                        'colour': colour_range,
                        'shape': shape,
                        'is_bin': is_bin,
                        'img_position': (cX,cY),
                        'global_position': global_position,
                        'rvec': rvecs[idx][0]
                        })

                # Combine matching objects that are very close together (likely double detections)
                combined_detections = []
                skip_indices = set()
                for i in range(len(detected)):
                    if i in skip_indices:
                        continue
                    obj1 = detected[i]
                    combined_det = obj1.copy()
                    for j in range(i+1, len(detected)):
                        if j in skip_indices:
                            continue
                        obj2 = detected[j]
                        if (obj1['shape'] == obj2['shape'] and
                            obj1['colour'] == obj2['colour']):
                            # Check distance between global positions
                            dist = np.linalg.norm(np.array(obj1['global_position']) - np.array(obj2['global_position']))
                            if dist < 0.03:  # Threshold distance to consider same object
                                skip_indices.add(j)
                                # Average the global positions
                                combined_det['global_position'] = tuple(
                                    (np.array(combined_det['global_position']) + np.array(obj2['global_position'])) / 2
                                )
                                combined_det['img_position'] = tuple(
                                    (np.array(combined_det['img_position']) + np.array(obj2['img_position'])) // 2
                                )
                    combined_detections.append(combined_det)
                detected = combined_detections
                
                # Output all detected objects from ArUco within this contour
                for det in detected:
                    rvec = det['rvec']
                    orientation = None # TODO: compute orientation for bin if needed using rvec
                    
                    if det['is_bin']:
                        goals.poses.append(self.make_goal(det['id'], det['colour'], det['shape'], det['global_position'], orientation))
                    else:
                        objects.poses.append(self.make_object(det['id'], det['colour'], det['shape'], det['global_position'], orientation))
                        
                    # Create and append marker for Rviz visualization
                    Marker_msg = self.make_marker(det['id'], det['colour'], det['global_position'], det['is_bin'], det['shape'], orientation)
                    markers.markers.append(Marker_msg)

                    # Draw the center on the original image for opencv visualization
                    cv2.circle(annotated, (det['img_position'][0], det['img_position'][1]), 5, (0, 0, 255), -1)
                    
            else:
                # No ArUco detected, classify shape normally    
                cX, cY, is_valid = self.get_contour_center(contour)
                # No ArUco detected, classify shape normally    
                if is_valid:
                    num_detected += 1        
                    shape, is_bin, _ = self.classify_shape(contour)
                    # Create and append marker for visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 255, 0), -1)
                    # cX,cY = 0,0
                    global_position = self.pixel_to_global([cX, cY], depth_offset=0.04 if is_bin else 0.02)
                    # convert rvec to orientation quaternion
                    # orientation = self.euler_to_quaternion(rvecs[idx][0])
                    if is_bin:
                        goals.poses.append(self.make_goal(num_detected, colour_range, shape, global_position, None))
                    else:
                        objects.poses.append(self.make_object(num_detected, colour_range, shape, global_position, None))
                        
                    # Create and append marker for Rviz visualization
                    Marker_msg = self.make_marker(num_detected, colour_range, global_position, is_bin, shape, None)
                    markers.markers.append(Marker_msg)

                    # Draw the center on the original image for opencv visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)

        # Pre-Aruco detection code here if needed
        # ---------------------------------------------------
        # # Loop through each contour to classify and locate objects
        # for colour_range, contour in contours:
        #     moments = cv2.moments(contour)
        #     if moments['m00'] != 0:
        #         # Calculate the center of the object
        #         cX = int(moments['m10'] / moments['m00'])
        #         cY = int(moments['m01'] / moments['m00'])
                
        #         # Convert the pixel coordinates to 3D world coordinates
        #         global_position = self.pixel_to_global([cX, cY])
        #         if global_position is not None:
        #             # Append the object to the list
        #             num_detected += 1
        #             shape, is_bin, _ = self.classify_shape(contour)
        #             orientation = None # TODO: compute orientation for bin if needed using marker detection/point cloud
        #             if is_bin:
        #                 goals.poses.append(self.make_goal(num_detected, colour_range, shape, global_position, orientation))
        #             else:
        #                 objects.poses.append(self.make_object(num_detected, colour_range, shape, global_position))
                        
        #             # Create and append marker for Rviz visualization
        #             Marker_msg = self.make_marker(num_detected, colour_range, global_position, is_bin, shape, orientation)
        #             markers.markers.append(Marker_msg)

        #             # Draw the center on the original image for opencv visualization
        #             cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)

        return goals, objects, markers, annotated, complete_mask

    # For vision demo only
    def test_objects(self):
        # Hardcoded test positions (in camera view) and colours
        test_objects = [{
            'position': [1.2, 0.4, -0.2],
            'colour': ObjectColour.RED,
            'shape': ObjectShape.CYLINDER,
            'is_bin': True
        },
        {
            'position': [1.2, 0.2, -0.2],
            'colour': ObjectColour.RED,
            'shape': ObjectShape.SPHERE,
            'is_bin': False
        },
        {
            'position': [1.2, -0.1, -0.2],
            'colour': ObjectColour.RED,
            'shape': ObjectShape.SPHERE,
            'is_bin': False
        },
        {
            'position': [1.2, -0.3, -0.2],
            'colour': ObjectColour.BLUE,
            'shape': ObjectShape.RECTANGULAR_PRISM,
            'is_bin': False
        },
        {
            'position': [1.2, 0.0, -0.2],
            'colour': ObjectColour.BLUE,
            'shape': ObjectShape.RECTANGULAR_PRISM,
            'is_bin': False
        }]
        
        # Create some test markers for visualization
        markers = MarkerArray()
        objects = LabelledPoseArray()
        objects.header.stamp = self.get_clock().now().to_msg()
        objects.header.frame_id = "camera_link"
        goals = LabelledPoseArray()
        goals.header.stamp = self.get_clock().now().to_msg()
        goals.header.frame_id = "camera_link"
        for idx, obj in enumerate(test_objects):
            marker_msg = self.make_marker(idx, obj['colour'], obj['position'], obj['is_bin'], obj['shape'])
            markers.markers.append(marker_msg)
            if idx == 0:
                goal = LabelledPose()
                goal.label = f"{obj['colour'].name}_{obj['shape'].name}_{idx+1}_goal"
                goal.colour = obj['colour'].name
                goal.shape = obj['shape'].name
                goal.pose.position = Point(x=obj['position'][0], y=obj['position'][1], z=obj['position'][2])
                goals.poses.append(goal)
                continue
            else:
                object = LabelledPose()
                object.label = f"{obj['colour'].name}_{obj['shape'].name}_{idx+1}"
                object.colour = obj['colour'].name
                object.shape = obj['shape'].name
                object.pose.position = Point(x=obj['position'][0], y=obj['position'][1], z=obj['position'][2])
                objects.poses.append(object)

        return goals, objects, markers

    def routine_callback(self):
        if (self.cv_image is None) and not IS_TEST:
            self.get_logger().info("No image received. Routine callback skipped.")
            return None

        goals, objects, markers, annotated, complete_mask = self.detect_objects()
        
        # For demo only without object detection
        if IS_TEST:
            goals, objects, markers = self.test_objects()
        # #

        # Publish detected objects
        self.object_pub.publish(objects)
        self.goal_pub.publish(goals)
        self.marker_pub.publish(markers)

        if annotated is not None and complete_mask is not None:
            cv2.imshow('annotated', annotated)
            cv2.imshow('complete_mask', complete_mask)
            cv2.waitKey(1)
        return

def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()

if __name__ == '__main__':
    main()