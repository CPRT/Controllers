import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_SPEED = 3.0     #2 m/s
ACC_REFRESH = 0.2  #updates speed with acceleration every 0.2

class thrustmaster_drive(Node):
    def __init__(self):
        super().__init__("thrustMasterDrive")
        self.twist = Twist()
        self.oldTime = 0.0
        self.currSpeed = 0.0
        
        self.setTwistPub = self.create_publisher(Twist, "TM_Pub", 1)
        self.cmd_move_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 10)

            
    def joy_callback(self, msg: Joy):
        #TH throttle min is -1 and max is 1
        if (msg.axes[3] < 0):
            self.acc = (1-(abs(msg.axes[3]))) / 2
        elif (msg.axes[3] > 0):
            self.acc = ((msg.axes[3]/2)+0.5)
        else:
            self.acc = 0.5
            
        if (msg.axes[1]<0): self.acc = -self.acc
        
        self.currTime = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        
        self.twist.angular.z = self.acc #placeholder too see acceleration 
        
        if (((self.currTime - self.oldTime) > ACC_REFRESH) & (msg.axes[1] >= 0)):
            if (self.currSpeed < msg.axes[1]*MAX_SPEED):
                self.currSpeed += self.acc
                self.twist.linear.x = self.currSpeed
            elif (self.currSpeed > msg.axes[1]*MAX_SPEED):
                self.twist.linear.x = msg.axes[1] * MAX_SPEED
                self.currSpeed = msg.axes[1] * MAX_SPEED
                
            self.oldTime = self.currTime
            self.setTwistPub.publish(self.twist)
            
        elif (((self.currTime - self.oldTime) > ACC_REFRESH) & (msg.axes[1] <= 0)):
            if (self.currSpeed > msg.axes[1]*MAX_SPEED):
                self.currSpeed += self.acc
                self.twist.linear.x = self.currSpeed
            elif (self.currSpeed < msg.axes[1]*MAX_SPEED):
                self.twist.linear.x = msg.axes[1] * MAX_SPEED
                self.currSpeed = msg.axes[1] * MAX_SPEED
                
            self.oldTime = self.currTime
            self.setTwistPub.publish(self.twist)
                
                
def main(args=None):
    rclpy.init(args=args)
    node = thrustmaster_drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()