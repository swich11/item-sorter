import rclpy

from rclpy.node import Node
from interfaces.msg import LabelledPoseArray, LabelledPose


class TestDetector(Node):
    def __init__(self):
        super().__init__('test_detector')
        self.object_pub = self.object_pub = self.create_publisher(LabelledPoseArray, "/camera/objects/labelled_pose_array", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        pose1 = LabelledPose()
        pose_goal = LabelledPose()
        pose1.label = "RedSquare"
        pose_goal.label = "SquareBucket"

        pose1.pose.position.x = 0.5
        pose1.pose.position.y = 0.4
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.w = 1.0
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = 0.0

        pose_goal.pose.position.x = 0.4
        pose_goal.pose.position.y = 0.5
        pose_goal.pose.position.z = 0.0
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0

        pose_array = LabelledPoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        pose_array.poses = [pose1, pose_goal]
        self.object_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = TestDetector()
    rclpy.spin(node)
    rclpy.shutdown()

