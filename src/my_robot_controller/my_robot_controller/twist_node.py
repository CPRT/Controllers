#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml

class TwistLimiterNode(Node):

    def __init__(self):
        super().__init__("twist_limiter")
        self.twist = Twist()
        self.max_linear = self.declare_parameter("max_speed", 1.0).value
        self.max_angular = self.declare_parameter("max_angular_speed", 1.0).value
        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.publisher = self.create_publisher(Twist, '/final_cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command) 
        self.get_logger().info("Twist Limiter node started.")        

    def twist_callback(self, msg):
        self.twist.linear.x = max(-self.max_linear, min(msg.linear.x, self.max_linear))
        self.twist.angular.z = max(-self.max_angular, min(msg.angular.z, self.max_angular))

    def send_velocity_command(self):
        self.publisher.publish(self.twist)
    
def main(args = None):
    rclpy.init(args = args)
    node = TwistLimiterNode()
    rclpy.spin(node)
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
