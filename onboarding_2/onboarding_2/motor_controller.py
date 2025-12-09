#Drive motor: 0x11
#Steering motor: 0x12

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can 
import struct
import math

class motorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        
        #declare parms
        self.declare_parameter("driver_motor_id", 0x11)
        self.declare_parameter("steering_motor_id", 0x12)
        
        #subscribe to car cmd from joystick
        self.cmd_vel_subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        self.timer_ = self.create_timer(0.25, self.paced_commands)
        
        #get parms
        self.driver_motor_id = self.get_parameter("driver_motor_id").value
        self.steering_motor_id = self.get_parameter("steering_motor_id").value
        
        #store vars
        self.v = 0.0        #velocity
        self.omega = 0.0    #angular

        self.max_speed = 1.0
        self.max_angle = math.radians(30)

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def paced_commands(self):
        velocity = self.compute_safe_velocity()
        angle = self.compute_safe_steering()

        drive_msg = self.create_drive_command(velocity)
        self.bus.send(drive_msg)

        steer_msg = self.create_steering_command(angle)
        self.bus.send(steer_msg)
        
    def compute_safe_velocity(self):
        return max(min(self.v, self.max_speed), -self.max_speed)
    
    def compute_safe_steering(self):
        return max(min(self.omega, self.max_angle), -self.max_angle)

    def create_drive_command(self, velocity = 0.0):
        priority = 0x00
        command_id = 0x03
        sender_node_id = 0x01

        arbitration_id = ( (priority << 24) | (command_id << 16) | (self.driver_motor_id << 8) | sender_node_id )

        data = struct.pack(">f", float(velocity))

        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)

        return message
    
    def create_steering_command(self, angle = 0.0):
        priority = 0x00
        command_id = 0x02
        sender_node_id = 0x01

        arbitration_id = ( (priority << 24) | (command_id << 16) | (self.steering_motor_id << 8) | sender_node_id)

        scaled_angle = float(angle) * -50.0

        data = struct.pack(">f", scaled_angle)

        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)

        return message

def main(args=None):
    rclpy.init(args=args)
    motor_node = motorControllerNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()