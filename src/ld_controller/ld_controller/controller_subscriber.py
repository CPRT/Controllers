import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import pyautogui as pgui
pgui.FAILSAFE = False

#def merge_coords(cur_x: int, cur_y: int) -> UInt32:
#    """
#    Takes the mouse's position on the screen in x and y
#    coordinates and converts them to a number, which is
#    composed of the coordinates binary. The first 16 bits
#    (left most ones) are the x coordinate and the last 16
#    bits are the y coordinate.
#    """

#    if cur_x <= 65537 or cur_y <= 65537:
#        pos_x = f'{cur_x:016b}'
#        pos_y = f'{cur_y:016b}'
#        return int(pos_x + pos_y, 2)
#    else:
#        return 0


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

        #message = '\nReceived:'
        #message += f'\n\t linear x: {rel_move_x}'
        #message += f'\n\t linear y: {rel_move_y}'
        #message += f'\n\t linear z: {cmd}'
        #message += f'\n\t angular y: {scroll}'
        #message += f'\n\t Screen x, y: {pgui.size()}'
        #self.get_logger().info(message)

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