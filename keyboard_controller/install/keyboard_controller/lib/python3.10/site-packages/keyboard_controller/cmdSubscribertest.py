#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cmdSubNode(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.node_subscriber = self.create_subscription(Twist, "/cmd_vel",self.twist_callback, 20)

    def twist_callback(self):
        self.get_logger().info('hello')



def main(args = None):
    rclpy.init(args=args)
    node = cmdSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()