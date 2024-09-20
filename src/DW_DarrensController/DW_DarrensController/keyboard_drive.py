from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

class keyboardDrive(Node):
    def __init__(self):
        super().__init__("KeyboardDriveNode")




        

def main(args=None):
    rclpy.init(args=args)
    node = keyboardDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()