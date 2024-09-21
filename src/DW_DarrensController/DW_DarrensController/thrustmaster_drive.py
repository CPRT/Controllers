import rclpy
import rclpy.logging
from rclpy.node import Node

from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class thrustMasterDrive(Node):
    def __init__(self):
        super().__init__("thrustMasterDrive_Node")
        self.twist = Twist()
        
        self.cmd_move_subscriber = self.create_subscription(Joy,"/joystick/drive", self.cmd_joy_callback, 10)

        self.setTwistPub = self.create_publisher(Twist, "/drive/cmd_vel", 1)
        
    def cmd_joy_callback(self, msg: Joy):
        self.twist = self.cmd_move_subscriber
        self.setTwistPub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = thrustMasterDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()