from enum import Enum

import cv2
import numpy as np

from ultralytics import YOLO
from ultralytics.engine.results import Results

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from interfaces.msg import LabelledPoseArray, LabelledPose # type: ignore


# these were widened because masks will only be taken for a bounding box
class ColourMasks(Enum):
    RED_LOW = ((0, 60, 0), (10, 255, 255))
    RED_HIGH = ((170, 60, 0), (180, 255, 255))
    GREEN = ((35, 60, 50), (95, 255, 255))
    BLUE = ((95, 150, 0), (115, 255, 255))

    @property
    def lower(self):
        return np.array(self.value[0], dtype=np.uint8)
    
    @property
    def upper(self):
        return np.array(self.value[1], dtype=np.uint8)
    

class ObjectColours(Enum):
    BLUE = 0
    GREEN = 1
    RED = 2


colour_dict = {
    "RedHexagon": ObjectColours.RED,
    "RedSquare": ObjectColours.RED,
    "RedCircle": ObjectColours.RED,
    "CircleBucket": ObjectColours.RED,

    "GreenHexagon": ObjectColours.GREEN,
    "GreenSquare": ObjectColours.GREEN,
    "GreenCircle": ObjectColours.GREEN,
    "HexagonBucket": ObjectColours.GREEN,

    "BlueHexagon": ObjectColours.BLUE,
    "BlueSquare": ObjectColours.BLUE,
    "BlueCircle": ObjectColours.BLUE,
    "SquareBucket": ObjectColours.BLUE,
}


class ItemDetector(Node):
    def __init__(self):
        super().__init__('item_detector')
        self._bridge = CvBridge()
        self._image_subscription = self.create_subscription(Image,
                                                      "/camera/camera/color/image_raw", 
                                                      self.image_callback,
                                                      10)
        self._point_cloud_sub = self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_img_callback, 10)
        self._camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.info_callback, 10)

        self.object_pub = self.create_publisher(LabelledPoseArray, "/camera/objects/labelled_pose_array", 10)

        self._image_publisher = self.create_publisher(Image, "/perception/detection/image", 10)
        self.model = YOLO("/home/julian/MTRN4231/item-sorter/sorter_ws/src/perception/resource/item-sorter.pt")

        self.fx = 150.0
        self.fy = 150.0
        self.ppx = 200
        self.ppy = 200
        self.depth_image = None
        self.got_intrinsics = False


    def info_callback(self, msg: CameraInfo) -> None:
        if self.got_intrinsics:
            return
        try:
            # Grab the intrinsics
            self.get_logger().info(f"Got camera instrinics: {msg.k}")
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.ppx = msg.k[2]
            self.ppy = msg.k[5]
        except CvBridgeError as e:
            self.get_logger().error(f"{e}")
            return
        self.got_intrinsics = True
        self.destroy_subscription(self._camera_info_sub)
        

    def image_callback(self, msg: Image) -> None:
        img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        results: Results = self.model(img, verbose=False)
        annotated_img = results[0].plot(show=False)
        annotated_img_msg = self._bridge.cv2_to_imgmsg(annotated_img)
        annotated_img_msg.header.frame_id = "camera_color_optical_frame"
        annotated_img_msg.header.stamp = self.get_clock().now().to_msg()
        self._image_publisher.publish(annotated_img_msg)
        if not self.got_intrinsics:
            return

        l_pose_array = LabelledPoseArray()
        l_pose_array.header.stamp = msg.header.stamp
        # l_pose_array.header.frame_id = "camera_depth_optical_frame"
        l_pose_array.header.frame_id = "camera_depth_optical_frame"


        if (self.depth_image is None):
            return

        # HSV image, we mask for segmentation
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for i, cls in enumerate(boxes.cls):
                    if boxes.conf[i] > 0.8:
                        l_pose = LabelledPose()
                        l_pose.label = result.names[int(cls)]
                        # crop image to bounding box area
                        x1, y1, x2, y2 = map(int, boxes.xyxy[i][:4])
                        bounding_img = hsv_img[y1:y2, x1:x2]
                        bounding_depth = self.depth_image[y1:y2, x1:x2]
                        # do masking to segment the image
                        match(colour_dict[l_pose.label]):
                            case ObjectColours.RED:
                                mask_low = cv2.inRange(bounding_img, ColourMasks.RED_LOW.lower, ColourMasks.RED_HIGH.upper)
                                mask_high = cv2.inRange(bounding_img, ColourMasks.RED_HIGH.lower, ColourMasks.RED_HIGH.upper)
                                mask = cv2.bitwise_or(mask_low, mask_high)
                            case ObjectColours.GREEN:
                                mask = cv2.inRange(bounding_img, ColourMasks.GREEN.lower, ColourMasks.GREEN.upper)
                            case ObjectColours.BLUE:
                                mask = cv2.inRange(bounding_img, ColourMasks.BLUE.lower, ColourMasks.BLUE.upper)
                        # average pixel positions in the mask + add to labelled pose
                        vs, us = np.where(mask)
                        Z = bounding_depth[vs, us]
                        l_pose.pose.position.x = np.average((x1 + us - self.ppx) * Z / self.fx) / 1000.0
                        l_pose.pose.position.y = np.average((y1 + vs - self.ppy) * Z / self.fy) / 1000.0
                        l_pose.pose.position.z = np.average(Z) / 1000.0
                        l_pose.pose.orientation.w = 0.924
                        l_pose.pose.orientation.x = -0.383
                        l_pose.pose.orientation.y = 0.0
                        l_pose.pose.orientation.z = 0.0
                        l_pose_array.poses.append(l_pose)
        self.object_pub.publish(l_pose_array) # just publish all objects in one array, this can be filtered by the brain


    def depth_img_callback(self, msg: Image) -> None:
        try:
            self.depth_image = self._bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in depth_img_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ItemDetector()
    executor = MultiThreadedExecutor(num_threads=2) # 1 thread for each callback running
    rclpy.spin(node)
    rclpy.shutdown()
