import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import pyautogui

class control_publisher(Node):
    def __init__(self):
        super().__init__("control_publisher")
        self.twist = Twist()

        self.setTwistPub = self.create_publisher(
            Twist, "/drive/cmd_vel", 1)
        self.cmd_move_subscribere = self.create_subscriber(
            Joy, "/joystick/drive", self.cmd_joy_callback, 10)

    def cmd_joy_callback(self, msg: Joy):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = control_publisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()