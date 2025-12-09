#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1
def main(args=None):
    rclpy.init(args=args)   #init ross communication

    #create node
    node = MyNode()
    rclpy.spin(node)    #contiune to run until we kill the node and allow for all the callbacks to run
    rclpy.shutdown()    #destroy node & shut down ross comms

if __name__ == '__main__':
    main()