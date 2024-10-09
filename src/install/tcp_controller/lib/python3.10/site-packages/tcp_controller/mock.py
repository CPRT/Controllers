import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class TcpMockery(Node):
    def __init__(self):
        super().__init__("tcp_forwarder")
        self.socket = socket.socket(socket.AF_INET)
        self.socket.connect(("127.0.0.1", 8228))

        run = True
        while run:
            match input():
                case "f": self.socket.send(b'\x00')
                case "r": self.socket.send(b'\x01')
                case "l": self.socket.send(b'\x02')
                case invalid: print(f"invalid command `{invalid}`")

def main(args=None):
    rclpy.init(args=args)
    node = TcpMockery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
