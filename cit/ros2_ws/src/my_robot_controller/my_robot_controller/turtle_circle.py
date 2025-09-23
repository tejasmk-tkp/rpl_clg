import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleCircle(Node):
    def __init__(self):
        super().__init__("turtle_circle")
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Turtle Circle Node has been started.")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: Linear X: %.2f, Angular Z: %.2f" % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    turtle_circle = TurtleCircle()
    rclpy.spin(turtle_circle)
    turtle_circle.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
