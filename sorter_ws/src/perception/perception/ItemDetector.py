from ultralytics.engine.results import Results
import cv2
import numpy as np

from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo


# Since I don't have access to a depth camera, positions are output as x, and y values
# in pixel coordinates of the centroid of the object
# if the depth camera was there, a relative position could instead be produced using
# the depth image of the camera
class ItemDetector(Node):
    def __init__(self):
        super().__init__('item_detector')
        self._bridge = CvBridge()
        self._subscription = self.create_subscription(Image,
                                                      "/camera/camera/color/image_raw", 
                                                      self.image_callback,
                                                      10)
        self._image_publisher = self.create_publisher(Image, "/perception/detection/image", 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.info_callback, 10)
        self.model = YOLO("/home/julian/MTRN4231/item-sorter/sorter_ws/src/perception/resource/item-sorter.pt")

        self.fx = 400.0
        self.fy = 400.0
        self.cx = 250.0
        self.cy = 250.0
        self.got_intrinsics = False


    def info_callback(self, msg: CameraInfo) -> None:
        intrinsics = msg.k
        self.get_logger().info(f"Got camera instrinics: {intrinsics}")
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.got_intrinsics = True
        # only need to gather camera intrinsics once
        self.destroy_subscription(self.camera_info_sub)
        

    def image_callback(self, msg: Image) -> None:
        img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        results: Results = self.model(img, verbose=False)
        annotated_img = results[0].plot(show=False)
        annotated_img_msg = self._bridge.cv2_to_imgmsg(annotated_img)
        annotated_img_msg.header.frame_id = "/camera_link" # fix that probs
        annotated_img_msg.header.stamp = self.get_clock().now().to_msg()
        self._image_publisher.publish(annotated_img_msg)

        for result in results:
            print(result.boxes)
            # masks = result.masks
            # if masks is not None:
            #     for i, mask in enumerate(masks):
            #         if result.names[int(result.boxes[i].cls[0])] == 'apple':
            #             binary_mask = mask.data.cpu().numpy().squeeze()
                        
            #             contours, _ = cv2.findContours(binary_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
            #             if contours:
            #                 largest_contour = max(contours, key=cv2.contourArea)
            #                 M = cv2.moments(largest_contour)
            #                 if M["m00"] != 0:
            #                     centroid_x = int(M["m10"] / M["m00"])
            #                     centroid_y = int(M["m01"] / M["m00"])

            #                     # publish a 3D position after we receive camera intrinsics
            #                     if (self.got_intrinsics):
            #                         point = Point()
            #                         # simulate depth imaging
            #                         z = (200**2 * 10)/(cv2.contourArea(largest_contour)) #just constant depth lol
                                    
            #                         point.x = float((centroid_x - self.cx)*z/self.fx)
            #                         point.y = float((self.cy - centroid_y)*z/self.fy)
            #                         point.z = float(z)

            #                         positions.positions.append(point)
            #     # Publish the pose array
            #     if len(positions.positions) > 0:
            #         positions.header.frame_id = 'camera_link'
            #         positions.header.stamp = self.get_clock().now().to_msg()
            #         self._poses_publisher.publish(positions)


def main(args=None):
    rclpy.init(args=args)
    node = ItemDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
