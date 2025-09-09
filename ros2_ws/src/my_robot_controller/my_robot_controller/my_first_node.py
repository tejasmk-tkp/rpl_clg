import rclpy
from rclpy.node import Node


class myNode(Node):
    def __init__(self):
        super().__init__("my_first_node")

def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
