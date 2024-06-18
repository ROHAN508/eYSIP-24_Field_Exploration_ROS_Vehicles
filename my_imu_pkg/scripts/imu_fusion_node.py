#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from madgwick_py import MadgwickAHRS

class ImuFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')
        self.subscription = self.create_subscription(Imu, 'imu/data_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.ahrs = MadgwickAHRS()
        
    def listener_callback(self, msg):
        self.ahrs.update_imu(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                             msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        fused_msg = Imu()
        fused_msg.header = msg.header
        fused_msg.orientation = Quaternion(*self.ahrs.quaternion)
        fused_msg.linear_acceleration = msg.linear_acceleration
        fused_msg.angular_velocity = msg.angular_velocity
        self.publisher_.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()