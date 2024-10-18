#!usr/bin/env python3

# credit to https://stackoverflow.com/questions/510357/how-to-read-a-single-character-from-the-user
# for user input code

import sys
import termios
import tty
import rclpy


from rclpy.node import Node
from geometry_msgs.msg import Twist




class controllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.publisher = self.create_publisher(Twist, "/controller", 20)
        self.linear_speed = 0.0  
        self.angular_speed = 0.0 
        self.x = 0.0
        self.z = 0.0
        self.declare_parameter('publish_freq', 0.5)
        
        self.accelType = 1
        self.timer = self.create_timer(0.5, self.twist_callback)
        self.get_logger().info("controller interface started")
        self.get_logger().info('Use w and s to move forward or backwards, use a and d to rotate, use q to reset speed, use 1, 2, and 3 to toggle between acceleration modes ')

        

    
    def twist_callback(self):
        
        twistMsg = Twist()
        print()
        self.getInput()
        twistMsg.linear.x = self.linear_speed
        print(self.linear_speed)
        twistMsg.angular.z = self.angular_speed
        print(self.angular_speed)
        self.publisher.publish(twistMsg)

    
    def getInput(self):
        print("Execute getInput")
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            print(ch)
            
            self.processInput(ch)
            print(self.linear_speed)
            print(self.angular_speed)
        except:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            exit()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
      
    def processInput(self, c):
        if c == 'w':
            self.wPress()
        if c == 'a':
            self.aPress()
        if c == 's':
            self.sPress()
        if c == 'd':
            self.dPress()
        if c == 'q':
            self.resetSpeed()
        if c == '0':
            self.changeAccel(0)
        if c == '1':
            self.changeAccel(1)
        if c == '2':
            self.changeAccel(2)
        if c == '3':
            self.changeAccel(3)

    

    def changeAccel(self, type: int):
        self.accelType = type
        print(self.accelType)
    
    def resetSpeed(self):
        self.x = 0.0
        self.z = 0.0
        self.angular_speed = 0.0
        self.linear_speed = 0.0

    def wPress(self):
        self.angular_speed = 0.0
        self.z = 0
        if (self.x <= 0.0):
            
            self.x = 0.0
        
        if (self.accelType >= 1):
            self.x += 0.05
        else:
            self.linear_speed = self.x
            return
        
        self.linear_speed = ((2.2*self.x) ** (self.accelType))/(2.2)
       

    def sPress(self):
        self.angular_speed = 0.0
        self.z = 0
        if (self.x >= 0):
            self.x = 0.0
        
        if (self.accelType >= 1):
                self.x -= 0.05
        else:
            self.linear_speed = self.x
            return
        if (self.accelType%2 == 0):
            self.linear_speed =(-1)* ((2.2*self.x) ** (self.accelType))/(2.2)
        else:
            self.linear_speed =((2.2*self.x) ** (self.accelType))/(2.2)

        

    def aPress(self):
        if (self.z <= 0):
            self.z = 0.0
            
        if (self.accelType >= 1):
            self.z += 0.07
        else:
            self.angular_speed = self.z
            return
        self.angular_speed = ((2.2*self.z) ** (self.accelType))/(2.2)
       

    def dPress(self):
        if (self.z >= 0):
            self.z = -0.15
        
        if (self.accelType >= 1):
            self.z -= 0.1
        else:
            self.angular_speed = self.z
            return
        if (self.accelType%2 == 0):
            self.angular_speed = (-1)* ((2.2*self.z) ** (self.accelType))/(2.2)
        else:
            self.angular_speed = ((2.2*self.z) ** (self.accelType))/(2.2)

       

def main(args = None):
    rclpy.init(args=args)
    node = controllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
