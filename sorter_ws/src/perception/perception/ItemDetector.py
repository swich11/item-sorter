import numpy as np
import pyrealsense2 as rs

from ultralytics import YOLO
from ultralytics.engine.results import Results

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from interfaces.msg import LabelledPoseArray, LabelledPose # type: ignore


# Since I don't have access to a depth camera, positions are output as x, and y values
# in pixel coordinates of the centroid of the object
# if the depth camera was there, a relative position could instead be produced using
# the depth image of the camera
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

        self.intrinsics = rs.intrinsics() # type: ignore
        self.depth_image = None
        self.got_intrinsics = False


    def info_callback(self, msg: CameraInfo) -> None:
        if self.got_intrinsics:
            return
        try:
            # Grab the intrinsics
            self.get_logger().info(f"Got camera instrinics: {msg.k}")
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            if msg.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady # type: ignore
            elif msg.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4 # type: ignore
            self.camera_matrix = np.array([
                [self.intrinsics.fx, 0, self.intrinsics.ppx],
                [0, self.intrinsics.fy, self.intrinsics.ppy],
                [0, 0, 1],
            ])
            self.dist_coeffs = np.array(self.intrinsics.coeffs)
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
        l_pose_array.header.frame_id = "camera_depth_optical_frame"


        if (self.depth_image is None):
            return


        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for i, cls in enumerate(boxes.cls):
                    if boxes.conf[i] > 0.9:
                        l_pose = LabelledPose()
                        l_pose.label = result.names[int(cls)]
                        cX = int(boxes.xywh[i][0])
                        cY = int(boxes.xywh[i][1])
                        # x, y, z position in camera frame
                        x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, (cX, cY), self.depth_image[cY,cX]*0.001) # type: ignore
                        l_pose.pose.position.x = x
                        l_pose.pose.position.y = y
                        l_pose.pose.position.z = z
                        l_pose.pose.orientation.w = 1.0
                        l_pose.pose.orientation.x = 0.0
                        l_pose.pose.orientation.y = 0.0
                        l_pose.pose.orientation.z = 0.0
                        l_pose_array.poses.append(l_pose)
        self.get_logger().info("Publishing labelled pose array.")
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
