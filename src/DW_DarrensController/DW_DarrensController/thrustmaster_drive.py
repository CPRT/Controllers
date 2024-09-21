import rclpy
import rclpy.logging
from rclpy.node import Node

from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class thrustmaster_drive(Node):
    def __init__(self):
        super().__init__("thrustMasterDrive")
        self.twist = Twist()
        
        self.cmd_move_subscriber = self.create_subscription(Joy, "joystick", self.cmd_joy_callback, 10)

        self.setTwistPub = self.create_publisher(Twist, "Joystick_pub", 1)
        
    def cmd_joy_callback(self, msg: Joy):
        #self.twist.linear.x = #axis
        self.setTwistPub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = thrustmaster_drive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()