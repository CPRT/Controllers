import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_ACC = 1         
ACC_REFRESH = 0.2  #updates speed with acceleration every 0.2

class thrustmaster_drive(Node):
    def __init__(self):
        super().__init__("thrustMasterDrive")
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.oldTime = 0.0
        self.currSpeed = 0.0
        self.maxSpeed = 3.0     #3 m/s
        self.buttonPressed = bool()
        self.buttonPressed = False
        
        self.setTwistPub = self.create_publisher(Twist, "TM_Pub", 1)
        self.cmd_move_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 10)

            
    def joy_callback(self, msg: Joy):
        #TH throttle min is -1 and max is 1
        if (msg.axes[3] < 0):
            self.acc = ((1-(abs(msg.axes[3]))) / 2) * MAX_ACC
        elif (msg.axes[3] > 0):
            self.acc = ((msg.axes[3]/2)+0.5) * MAX_ACC
        else:
            self.acc = 0.5 * MAX_ACC
            
        if (msg.axes[1]<0): self.acc = -self.acc
        
        self.currTime = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        self.twist.angular.z = self.acc #placeholder too see acceleration 
        
        #forwards
        if (((self.currTime - self.oldTime) > ACC_REFRESH) and (msg.axes[1] >= 0)):
            if ((self.currSpeed < msg.axes[1]*self.maxSpeed) and (self.currSpeed + self.acc < msg.axes[1]*self.maxSpeed)):
                self.currSpeed += self.acc
                self.twist.linear.x = self.currSpeed
            elif (self.currSpeed > msg.axes[1]*self.maxSpeed):
                self.twist.linear.x = msg.axes[1] * self.maxSpeed
                self.currSpeed = msg.axes[1] * self.maxSpeed
            else:
                self.currSpeed += (msg.axes[1]*self.maxSpeed)-self.currSpeed
                self.twist.linear.x = self.currSpeed
                
            self.oldTime = self.currTime
            self.setTwistPub.publish(self.twist)
            
        #reverse
        elif (((self.currTime - self.oldTime) > ACC_REFRESH) and (msg.axes[1] <= 0)):
            if ((self.currSpeed > msg.axes[1]*self.maxSpeed) and (self.currSpeed + self.acc > msg.axes[1]*self.maxSpeed)):
                self.currSpeed += self.acc
                self.twist.linear.x = self.currSpeed
            elif (self.currSpeed < msg.axes[1]*self.maxSpeed):
                self.twist.linear.x = msg.axes[1] * self.maxSpeed
                self.currSpeed = msg.axes[1] * self.maxSpeed
            else:
                self.currSpeed += (msg.axes[1]*self.maxSpeed)-self.currSpeed
                self.twist.linear.x = self.currSpeed
                
            self.oldTime = self.currTime
            self.setTwistPub.publish(self.twist)
            
        # THRUSTMASTER BUTTON SCHEME
        #------------------------------------
        #     12    |     11     |    10    |
        #------------------------------------
        #     13    |     14     |    15    |
        #------------------------------------
        #
        #------------------------------------
        # MAX SPD + | MAX SPD - |   Broken  |
        #------------------------------------
        #   E-STOP  |    TBD    |    TBD    |
        #------------------------------------
        #
        
        #MAX SPD +
        if (msg.buttons[12] == 1 and self.buttonPressed == False):
            self.buttonPressed = True
            self.maxSpeed += 0.2
        elif (msg.buttons[12] == 0):
            self.buttonPressed = False
            
        #MAX SPD -
        if (msg.buttons[11] == 1 and self.buttonPressed == False):
            self.buttonPressed = True
            self.maxSpeed -= 0.2
        elif (msg.buttons[11] == 0):
            self.buttonPressed = False
            
        #E-STOP (will have to restart to continue driving)
        if (msg.buttons[13] == 1 and self.buttonPressed == False):
            self.buttonPressed = True
            self.maxSpeed = 0.0
                
                
def main(args=None):
    rclpy.init(args=args)
    node = thrustmaster_drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()