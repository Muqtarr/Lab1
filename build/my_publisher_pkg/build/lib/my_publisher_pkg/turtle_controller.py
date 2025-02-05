import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn, TeleportAbsolute
from turtlesim.msg import Pose
import math
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.pose = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle_MUKHTARVALERIYAAZHARDARYN/cmd_vel', 10)
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle_MUKHTARVALERIYAAZHARDARYN/teleport_absolute')
        self.create_subscription(Pose, '/turtle_MUKHTARVALERIYAAZHARDARYN/pose', self.pose_callback, 10)

        self.kill_turtle('turtle1')
        self.spawn_turtle('turtle_MUKHTARVALERIYAAZHARDARYN', 5.5, 5.5, 0.0)
        self.wait_for_pose()
        self.teleport_turtle(1.0, 1.0, 0.0)
        self.wait_for_pose()
        self.follow_coordinates([(10, 10), (1, 10), (1, 1), (10, 1), (10, 10)])
    
    def pose_callback(self, msg):
        self.pose = msg

    def wait_for_pose(self):
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_request(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def kill_turtle(self, name):
        self.send_request(self.kill_client, Kill.Request(name=name))

    def spawn_turtle(self, name, x, y, theta):
        self.send_request(self.spawn_client, Spawn.Request(name=name, x=x, y=y, theta=theta))

    def teleport_turtle(self, x, y, theta):
        self.send_request(self.teleport_client, TeleportAbsolute.Request(x=x, y=y, theta=theta))

    def move_to(self, x, y):
        if not self.pose:
            return
        angle = math.atan2(y - self.pose.y, x - self.pose.x)
        self.rotate_to(angle)
        distance = math.hypot(x - self.pose.x, y - self.pose.y)
        self.move_straight(distance)

    def rotate_to(self, angle):
        if not self.pose:
            return
        msg = Twist()
        while abs(angle - self.pose.theta) > 0.01:
            msg.angular.z = 0.8 * (1 if angle > self.pose.theta else -1)
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.cmd_vel_pub.publish(Twist())

    def move_straight(self, distance):
        if not self.pose:
            return
        msg = Twist()
        msg.linear.x = 1.0
        start_x, start_y = self.pose.x, self.pose.y
        while math.hypot(self.pose.x - start_x, self.pose.y - start_y) < distance:
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.cmd_vel_pub.publish(Twist())

    def follow_coordinates(self, coordinates):
        for x, y in coordinates:
            self.move_to(x, y)

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
