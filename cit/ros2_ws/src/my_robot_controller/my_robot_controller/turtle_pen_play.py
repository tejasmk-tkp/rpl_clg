#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_services')

        # Publisher to control turtle velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Client to change pen color
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info("Turtle controller has been started.")

    def pose_callback(self, pose: Pose):
        cmd = Twist()

        # Movement control logic — bounce off the walls
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)

        # Change pen color based on turtle position
        if pose.x > 5.5:
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        else:
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0, 255, 0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        # Ensure service is ready
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /turtle1/set_pen service...")

        # Create and send request
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        self.pen_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
