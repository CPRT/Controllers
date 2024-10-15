import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import pyautogui

class control_subscriber(Node):
    def __init__(self):
        super().__init__("control_subsciber")
        self.move_x = 0
        self.move_y = 0
        self.current_x, self.current_y = pyautogui.position()

        self.setTwistPub = self.create_subscriber(
            Twist, "/drive/cmd_vel",self.move_mouse, 1)
        
    def move_mouse(self, twist):
        self.move_x = twist.linear.x
        self.move_y = twist.linear.y
        new_x = self.current_x + self.move_x
        new_y = self.current_y + self.move_y

        if not pyautogui.onScreen(new_x, new_y):
            return None
        
        pyautogui.moveTo(new_x, new_y)






def main(args=None):
    rclpy.init(args=args)
    node = control_subscriber()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()