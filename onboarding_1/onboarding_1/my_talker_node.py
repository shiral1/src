#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class talkerNode(Node):
    def __init__(self):
        super().__init__("my_talker_node")
        self.talk = self.create_publisher(String, "chatter", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World!"
        self.talk.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        
def main(args=None):
    rclpy.init(args=args)
    node = talkerNode()
    rclpy.spin(node)
    rclpy.shutdown()