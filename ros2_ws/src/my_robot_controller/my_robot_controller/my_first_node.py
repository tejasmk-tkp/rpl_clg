import rclpy
from rclpy.node import Node


class myNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = myNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
