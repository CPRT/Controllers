import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Bool, String


from geometry_msgs.msg import Twist

import sys, select, termios, tty

class key_reader(Node):
    def __init__(self):
        super().__init__("keyboard")
        self.setting = termios.tcgetattr(sys.stdin)
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.out = Twist()
        self.timer = self.create_timer(period, self.getKey)
        self.setKeyPub = self.create_publisher(
            String, "/key", 1)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)
        self.setKeyPub.publish(String(data=key))
        print(key)
        if key == ".":
            exit("Code Killed")

def main(args=None): 
    rclpy.init(args=args)
    node = key_reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()