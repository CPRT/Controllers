#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SubscriberPublisherNode(Node):
    def __init__(self):
        super().__init__("subscribe_and_publish")
        self.linear = 0.0
        self.angular = 0.0 
        
        self.declare_parameter('publish_freq', 0.5)
        self.declare_parameter('linear_speed_limit', 1.7)
        self.declare_parameter('angular_speed_limit', 1.3)
        
        self.publish_freq =  self.get_parameter('publish_freq').value
        self.linear_limit = self.get_parameter('linear_speed_limit').value
        self.angular_limit =  self.get_parameter('angular_speed_limit').value
        self.node_subscriber = self.create_subscription(Twist, "/controller",self.twist_callback, 20)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 20)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    
    def twist_callback(self, msg:Twist):
        self.get_logger().info('Subscriber message')
        if (msg.linear.x > self.linear_limit):
            self.linear = self.linear_limit
        elif (msg.linear.x < (-1)*self.linear_limit):
            self.linear = (-1)*self.linear_limit
        else:
            self.linear = msg.linear.x
        if (msg.angular.z > self.angular_limit):
            self.angular = self.angular_limit
        elif (msg.angular.z < (-1)*self.angular_limit):
            self.angular = (-1)*self.angular_limit
        else: 
            self.angular = msg.angular.z
    
    def timer_callback(self):
        twistMsg = Twist()
        twistMsg.linear.x = self.linear
        twistMsg.angular.z = self.angular
        self.get_logger().info("(" + str(twistMsg.linear.x) +", " + str(twistMsg.angular.z) + ")")
        self.publisher.publish(twistMsg)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
