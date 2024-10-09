import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

MESSENGER_NETLOC = ("127.0.0.1", 8228)

class TcpForwarder(Node):
    def __init__(self):
        super().__init__("tcp_forwarder")
        self.forwarder = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.socket = socket.socket(socket.AF_INET)
        self.socket.bind(MESSENGER_NETLOC)
        self.socket.listen()
        self.get_logger().info("awaiting TCP connection...")
        self.stream = self.socket.accept()[0]
        self.stream.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.stream.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.get_logger().info("ready to receive commands")

        run = True
        while run:
            b = self.stream.recv(1)
            self.get_logger().info(f"received {b}")
            if not b: run = False; break
            self.process(b[0])
        print("connection terminated, cleaning up")
        self.stream.close()
        self.socket.close()
    
    def process(self, cmd: int):
        twist = Twist()
        match cmd:
            case 0: twist.linear.x = 1.0
            case 1: twist.angular.z = -1.0
            case 2: twist.angular.z = 1.0
            case 3: pass
            case invalid: self.get_logger().warn(f"discarding invalid command identifier {invalid}"); return
        self.forwarder.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TcpForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
