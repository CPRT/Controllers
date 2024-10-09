#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")" + str(self.counter_))
        self.counter_ += 1

    def timer_callback(self):
        # Log a message every second (you can update this logic as needed)
        self.get_logger().info(f"Timer triggered, Counter: {self.counter_}")
        
def main(args = None):
    rclpy.init(args = args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()