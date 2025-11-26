#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.publisher = self.create_publisher(String, '/arduino_cmds', 10)
        self.timer = self.create_timer(5.0, self.publish_cmd)
        self.state = False

    def publish_cmd(self):
        msg = String()
        msg.data = "close\n" if self.state else "open\n"
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.state = not self.state

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
