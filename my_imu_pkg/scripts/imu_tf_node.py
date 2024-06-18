#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class ImuTFNode(Node):
    def __init__(self):
        super().__init__('imu_tf_node')
        self.subscription = self.create_subscription(Imu, 'imu/data', self.listener_callback, 10)
        self.br = TransformBroadcaster(self)
        
    def listener_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.rotation = msg.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
