#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

key_mapping = {
    'w': [0.1, 0],    # Increase forward speed
    's': [-0.1, 0],   # Decrease forward speed
    'a': [0, 0.1],    # Increase angular speed
    'd': [0, -0.1],   # Decrease angular speed
    'r': [0, 0]       # Reset speeds
}

class KeyboardInterfaceNode(Node):
    def __init__(self):
        super().__init__('keyboard_interface')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.0  # Initialize linear speed
        self.angular_speed = 0.0  # Initialize angular speed

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        global settings
        settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                key = self.get_key()
                if key in key_mapping:
                    # Update speeds based on key pressed
                    change = key_mapping[key]
                    self.linear_speed += change[0]  # Update linear speed
                    self.angular_speed += change[1]  # Update angular speed

                    # Create and publish the Twist message
                    twist = Twist()
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed
                    self.pub.publish(twist)

                    # Reset speeds when 'r' is pressed
                    if key == 'r':
                        self.linear_speed = 0.0
                        self.angular_speed = 0.0
                        self.get_logger().info("Speeds reset to 0.")
                if key == '\x03':  # Exit on Ctrl+C
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInterfaceNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
