#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from read_all_data import BMX160
import sys
sys.path.append('../../')
import time
from DFRobot_BMX160 import BMX160

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        self.bmx = BMX160(bus=1)
        self.bmx.begin()

    def publish_imu_data(self):
        data = self.bmx.get_all_data()
        msg = Imu()
        msg.linear_acceleration.x = data[6]
        msg.linear_acceleration.y = data[7]
        msg.linear_acceleration.z = data[8]
        msg.angular_velocity.x = data[3]
        msg.angular_velocity.y = data[4]
        msg.angular_velocity.z = data[5]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
