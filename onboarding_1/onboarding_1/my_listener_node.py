#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class listenerNode(Node):
    def __init__(self):
        super().__init__("my_listener_node")
        self.listener = self.create_subscription(String, "chatter", self.callback, 10)

    def callback(self, msg: String):
        self.get_logger().info(f"Recieved: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = listenerNode()
    rclpy.spin(node)
    rclpy.shutdown()