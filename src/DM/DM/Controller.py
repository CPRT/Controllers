import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Bool, String


from geometry_msgs.msg import Twist

import sys, select, termios, tty, os

class keyboard(Node):
    def __init__(self):
        super().__init__("keyboard")
        # os.set_blocking(sys.stdin.fileno(), False)
        self.setting = termios.tcgetattr(sys.stdin)
        self.Max_speed = 0.50
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.forward = 50
        self.turn = 50
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.key= "test"
        self.out = Twist()
        self.getkey = self.create_subscription(String,"/key",self.gkey,1)
        self.timer = self.create_timer(period, self.keyboard_inputs)
        # self.timer2 = self.create_timer(period, self.getKey)
        self.setTwistPub = self.create_publisher(
            Twist, "/drive/cmd_vel", 1)
        self.setKeyPub = self.create_publisher(
            String, "/key", 1)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        helps = select.select([sys.stdin], [], [], 0.1)
        if helps[0] == []:
            return
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)
        self.setKeyPub.publish(String(data=key))
        print(key)

    def gkey(self, msg: String):
        self.key = msg.data

    def keyboard_inputs(self):
        self.getKey()
        if self.key== 'w':
            if self.linear_speed < self.Max_speed:
                self.linear_speed = self.Max_speed/self.forward
                self.forward -= 1
        if self.key== 'a':
            if self.angular_speed < self.Max_speed and self.turn != 0:
                self.angular_speed = self.Max_speed/self.turn
                self.turn -=1
            elif self.angular_speed > -self.Max_speed:
                self.turn -=1
                self.angular_speed = -self.Max_speed/self.turn
        if self.key== 'd':
            if self.angular_speed > -self.Max_speed and self.turn != 0:
                self.angular_speed = -(self.Max_speed/self.turn)
                self.turn -=1
            elif self.angular_speed > -self.Max_speed:
                self.turn -=1
                self.angular_speed = -self.Max_speed/self.turn
        if self.key== 's':  
            if self.linear_speed <= self.Max_speed and self.forward !=0:
                self.linear_speed = self.Max_speed/self.forward
                self.forward += 1
            else:
                self.linear_speed = 0.0
                self.forward = 0
        if self.key== ' ':
            self.speed = 0.0
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.turn = 5
            self.forward = 5
        if self.key== '1':
            self.Max_speed = 0.0
        if self.key== '2':
            self.Max_speed = 1.5
        if self.key== '3':
            self.Max_speed = 2.0
        if self.key== '.':
            exit("Code Killed")
        #print(self.forward)
        self.out.linear.x = self.linear_speed
        self.out.angular.z = self.angular_speed
        print(self.out)
        #print(self.key)
        self.setTwistPub.publish(self.out)

    def test(self):
        print("test")


def main(args=None):
    rclpy.init(args=args)
    node = keyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
