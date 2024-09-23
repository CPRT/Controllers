import rclpy
import rclpy.logging
from rclpy.node import Node

from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_SPEED = 2.0     #2 m/s
MAX_ACC = 
MIN_ACC = 

class thrustmaster_drive(Node):
    def __init__(self):
        super().__init__("thrustMasterDrive")
        self.twist = Twist()
        
        self.setTwistPub = self.create_publisher(Twist, "TM_Pub", 1)
        self.cmd_move_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        
    def joy_callback(self, msg: Joy):
        
        
        self.twist.linear.x = msg.axes[1]
        self.twist.linear.z = msg.axes[0]
        self.setTwistPub.publish(self.twist)
        

def main(args=None):
    rclpy.init(args=args)
    node = thrustmaster_drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()