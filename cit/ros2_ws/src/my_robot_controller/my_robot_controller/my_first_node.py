import rclpy
from rclpy.node import Node


class myNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.get_logger().info("Hello ROS2")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("This is my first ROS2 node!")

def main(args=None):
    rclpy.init(args=args)
    node = myNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
