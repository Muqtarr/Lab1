#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveNode(Node):
    def __init__(self):
        super().__init__("move_turtle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Moving node started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    rclpy.shutdown()