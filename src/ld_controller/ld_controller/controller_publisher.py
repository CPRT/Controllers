import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import pyautogui as pgui

def joystick_to_mouse(value, size, throttle, sensitivity = 1):
    if throttle == False:
        scaling_factor = (size//350) * sensitivity
    else:
        scaling_factor = 2
    return - float(int(value * scaling_factor))

class control_publisher(Node):
    def __init__(self):
        super().__init__("control_publisher")
        self.twist = Twist()
        self.cur_x, self.cur_y = pgui.position()
        self.size_x, self.size_y = pgui.size()
        self.throttle = False

        if self.size_x <= self.size_y:
            self.size = self.size_x
        else:
            self.size = self.size_y

        self.setTwistPub = self.create_publisher(
            Twist, "commands", 1)
        self.cmd_move_subscribere = self.create_subscription(
            Joy, "/joy", self.cmd_joy_callback, 10)
        
        freq = 144
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.pub)

    def pub(self):
        self.twist.linear.x = float(self.cur_x)
        self.twist.linear.y = float(self.cur_y)
        self.setTwistPub.publish(self.twist)

    def cmd_joy_callback(self, msg: Joy):
        if msg.buttons[7] == 1:
            self.throttle = True
        else:
            self.throttle = False

        new_x = self.cur_x + joystick_to_mouse(msg.axes[0], self.size, self.throttle)
        new_y = self.cur_y + joystick_to_mouse(msg.axes[1], self.size, self.throttle)

        #message = '\nReceived:'
        #message += f'\n\t old x: {self.cur_x}'
        #message += f'\n\t old y: {self.cur_y}'
        #message += f'\n\t new x: {new_x}'
        #message += f'\n\t new y: {new_y}'
        #message += f'\n\t Screen x, y: {pgui.size()}'
        #self.get_logger().info(message)

        if pgui.onScreen(new_x, new_y):
            # move mouse
            self.cur_x = new_x
            self.cur_y = new_y
            #self.twist.linear.x += joystick_to_mouse(msg.axes[0], self.size)
            #self.twist.linear.y += joystick_to_mouse(msg.axes[1], self.size)

        self.twist.linear.z = 0.0
        if msg.buttons[6] == 1 and msg.buttons[1] == 1:
            # double left click
            self.twist.linear.z = 0.5
        elif msg.buttons[1] == 1: 
            # left click
            self.twist.linear.z = 0.25
        elif msg.buttons[0] == 1:
            # right click
            self.twist.linear.z = 0.75
        elif msg.buttons[2] == 1:
            # middle click
            self.twist.linear.z = 1.0

        # scrolling
        self.twist.angular.y = - joystick_to_mouse(msg.axes[3], self.size, self.throttle)

    # Things that mouse needs to be able to do:
        # - move (linear.x and .y, left joystick)
        # - left click (linear.z = 1, press A)
        # - right click (linear.z = 3, press B)
        # - drag and drop (not implemented)
        # - scroll up and down (angular.x and .y, right joystick up and down)
    
    # Things that mouse could try to be able to do:
        # - zoom in and out
        # - middle click (linear.z = 4, press X)
        # - double click (linear.z = 2, press right paddle + A)
        # - diff sticks for different refinement of controls

def main(args=None):
    rclpy.init(args=args)
    node = control_publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()