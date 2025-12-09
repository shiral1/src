#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joystickInterpreter(Node):
    def __init__(self):
        super().__init__("joystick_interpreter")

        self.callback_group = ReentrantCallbackGroup()

        #declare parms for joystick mapping
        self.declare_parameter("velocity_axis", 1)
        self.declare_parameter("steering_axis", 3)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("deadzone", 0.1)

        #subscribe to joystick input
        self.joy_subscription = self.create_subscription(Joy, "joy1", self.joy_callback, 10, callback_group=self.callback_group)
        self.car_cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10, callback_group=self.callback_group)
        self.timer_ = self.create_timer(1.0, self.check_connection, callback_group=self.callback_group)
        # track connection
        self.connected = False
        self.last_received_time = self.get_clock().now()

        #get parms
        self.velocity_axis = self.get_parameter("velocity_axis").value
        self.steering_axis = self.get_parameter("steering_axis").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.deadzone = self.get_parameter("deadzone").value

    def apply_deadzone(self, value):
        """Applying deadzone to joystick input to account for drift"""
        if abs(value) < self.deadzone:
            return 0.0
        
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled
    
    def joy_callback(self, msg):
        """ Process joystick inputs and publish commands"""
        self.connected = True
        self.last_received_time = self.get_clock().now()

        if len(msg.axes) > max(self.velocity_axis, self.steering_axis):

            velocity_input = msg.axes[self.velocity_axis]
            steering_input = msg.axes[self.steering_axis]

            velocity_input = self.apply_deadzone(velocity_input)
            steering_input = self.apply_deadzone(steering_input)

            linear_speed = velocity_input * self.max_linear_speed
            angular_speed = -steering_input * self.max_angular_speed

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            self.car_cmd_publisher.publish(twist)
        
    def check_connection(self):
        current_time = self.get_clock().now()
        if self.connected and (current_time - self.last_received_time).nanoseconds() > 5.0 * 1e9:   #No message for 5 sec
            self.get_logger().warn("No controller input received recently. Is the controller connected?")
            self.connected = False
        elif not self.connected:
            self.get_logger().warn("Waiting for controller input...")
            
def main(args=None):
    rclpy.init(args=args)
    joystick_node = joystickInterpreter()
    executor = MultiThreadedExecutor()
    rclpy.spin(joystick_node, executor=executor)
    joystick_node.destroy_node()
    rclpy.shutdown()