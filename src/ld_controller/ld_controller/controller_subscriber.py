import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import pyautogui as pgui
pgui.FAILSAFE = False # Turn failsafe off to avoid getting stuck in corners

async def mouse_move(x, y):
    pgui.moveTo(x, y, 0)

async def command(cmd):
    if not(cmd == 0):
        if cmd == 0.25:
            pgui.leftClick()
        elif cmd == 0.5:
            pgui.doubleClick()
        elif cmd == 0.75:
            pgui.rightClick()
        elif cmd == 1.0:
            pgui.middleClick()

async def scrolling(scroll):
    if not(scroll == 0):
        pgui.scroll(scroll)

class control_subscriber(Node):
    def __init__(self):
        super().__init__("control_subsciber")

        self.setTwistSub = self.create_subscription(
            Twist, "commands", self.execute_cmd, 1)

    async def execute_cmd(self, msg: Twist):
        x = msg.linear.x
        y = msg.linear.y
        scroll = msg.angular.y
        cmd = msg.linear.z

        await mouse_move(x, y)
        await command(cmd)
        await scrolling(scroll)

def main(args=None):
    rclpy.init(args=args)
    node = control_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()