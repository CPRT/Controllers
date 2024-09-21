import rclpy
from rclpy.node import Node

import sys
from std_msgs.msg import String

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class keyboardListener(Node):
    
    def __init__(self):
        super().__init__("keyboard_controls")    
        self.publisher_ = self.create_publisher(String, 'keys', 10)
        
        if sys.platform == 'win32':
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin)) #might not need this
        self.publisher_.publish(key)
        
        
def main(args=None):
    
    rclpy.init(args=args)
    node = keyboardListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()