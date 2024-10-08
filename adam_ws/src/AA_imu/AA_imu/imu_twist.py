#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class ConvertRaw(Node):
    def __init__(self):
        super().__init__("imu_twist")

        # config from yaml
        self.publish_rate = self.declare_parameter("~publish_rate", 1).value
        self.imu_pub_topic = self.declare_parameter("~imu_pub_topic", "/imu_msg/raw").value
        self.twist_pub_topic = self.declare_parameter("~twist_pub_topic", "/cmd_vel").value
        self.max_speed = self.declare_parameter("~max_speed", 10.0).value
        self.max_angular_speed = self.declare_parameter("~max_angular_speed", 10.0).value
        self.jitter = self.declare_parameter("~jitter", 0.1).value
        
        # publish and subscribe
        self.cmd_vel_pub = self.create_publisher(Twist, self.twist_pub_topic, 10)
        self.imu_raw_sub = self.create_subscription(Imu, self.imu_pub_topic, self.raw_read, 10)

        # publish rate
        self.timer = self.create_timer(float(1/self.publish_rate), self.convert_and_send)

        # cache last data
        self.last_imu_msg = Imu()

        self.get_logger().info(f"IMU raw to twist converter started. Rate: {self.publish_rate}")

    def sign(self, a):
        return -1 if a < 0 else 1

    # called when our publish timer fires
    def convert_and_send(self):
        # convert the last msg and send
        twist_msg = Twist()

        # The imu has linear acceleration values
        # twist is velocity
        # for now a quick 1 to 1 mapping
        
        # do not exceed the max speed values
        # for now one max is specifed for linear and one for angular
        if abs(self.last_imu_msg.linear_acceleration.x) <= self.max_speed:
            twist_msg.linear.x = self.last_imu_msg.linear_acceleration.x
        else:
            twist_msg.linear.x = self.max_speed * self.sign(self.last_imu_msg.linear_acceleration.x)
            
        if abs(self.last_imu_msg.linear_acceleration.y) <= self.max_speed:
            twist_msg.linear.y = self.last_imu_msg.linear_acceleration.y
        else:
            twist_msg.linear.y = self.max_speed * self.sign(self.last_imu_msg.linear_acceleration.y)
            
        if abs(self.last_imu_msg.linear_acceleration.z) <= self.max_speed:
            twist_msg.linear.z = self.last_imu_msg.linear_acceleration.z
        else:
            twist_msg.linear.z = self.max_speed * self.sign(self.last_imu_msg.linear_acceleration.z)
            
        if abs(self.last_imu_msg.angular_velocity.x) <= self.max_angular_speed:
            twist_msg.angular.x = self.last_imu_msg.angular_velocity.x
        else:
            twist_msg.angular.x = self.max_angular_speed * self.sign(self.last_imu_msg.angular_velocity.x)
            
        if abs(self.last_imu_msg.angular_velocity.y) <= self.max_angular_speed:
            twist_msg.angular.y = self.last_imu_msg.angular_velocity.y
        else:
            twist_msg.angular.y = self.max_angular_speed * self.sign(self.last_imu_msg.angular_velocity.y)
            
        if abs(self.last_imu_msg.angular_velocity.z) <= self.max_angular_speed:
            twist_msg.angular.z = self.last_imu_msg.angular_velocity.z
        else:
            twist_msg.angular.z = self.max_angular_speed * self.sign(self.last_imu_msg.angular_velocity.z)

        # filter out the very small values (jitter)
        if abs(self.last_imu_msg.linear_acceleration.x) >= self.jitter:
            twist_msg.linear.x = self.last_imu_msg.linear_acceleration.x
        else:
            twist_msg.linear.x = 0.0
            
        if abs(self.last_imu_msg.linear_acceleration.y) >= self.jitter:
            twist_msg.linear.y = self.last_imu_msg.linear_acceleration.y
        else:
            twist_msg.linear.y = 0.0
            
        if abs(self.last_imu_msg.linear_acceleration.z) >= self.jitter:
            twist_msg.linear.z = self.last_imu_msg.linear_acceleration.z
        else:
            twist_msg.linear.z = 0.0
            
        if abs(self.last_imu_msg.angular_velocity.x) >= self.jitter:
            twist_msg.angular.x = self.last_imu_msg.angular_velocity.x
        else:
            twist_msg.angular.x = 0.0
            
        if abs(self.last_imu_msg.angular_velocity.y) >= self.jitter:
            twist_msg.angular.y = self.last_imu_msg.angular_velocity.y
        else:
            twist_msg.angular.y = 0.0
            
        if abs(self.last_imu_msg.angular_velocity.z) >= self.jitter:
            twist_msg.angular.z = self.last_imu_msg.angular_velocity.z
        else:
            twist_msg.angular.z = 0.0

        # publish the message
        self.cmd_vel_pub.publish(twist_msg)


    # called when the imu_node sends a message
    # cache the message
    def raw_read(self, msg: Imu):
        self.last_imu_msg = msg

def main():
    rclpy.init(args=None)
    node = ConvertRaw()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
