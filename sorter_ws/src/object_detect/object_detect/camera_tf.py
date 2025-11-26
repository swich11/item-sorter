import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `base_link` to a static camera frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, transformation):
        super().__init__('camera_frame_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = transformation[0]

        t.transform.translation.x = float(transformation[1])
        t.transform.translation.y = float(transformation[2])
        t.transform.translation.z = float(transformation[3])
        quat = quaternion_from_euler(
            float(transformation[4]), float(transformation[5]), float(transformation[6]))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)

def main():
    logger = rclpy.logging.get_logger('logger')

    # NEED TO BE VERIFIED FOR THE SETUP
    # Potentially changed to parameters later
    x = 1.2
    y = 0.0
    z = 1.0Z
    pitch = (45.0/180.0)*math.pi
    yaw = math.pi

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher(['camera_frame',x,y,z,roll,pitch,yaw])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()