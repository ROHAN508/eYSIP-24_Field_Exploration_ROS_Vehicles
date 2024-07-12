import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D , TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros

# Global variable to keep track of initial position
k = 0

class PoseEstimator(Node):
    def __init__(self):
        # Initialize the node with the name 'vicon_odom_node'
        super().__init__('vicon_odom_node')
        
        # Subscription to PoseStamped messages from the Vicon system
        self.sub_pwm = self.create_subscription(PoseStamped, '/vicon/RCCar/RCCar/pose', self.callback, 10)
        
        # Publisher for the Odometry messages
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Initialize the Odometry message
        self.Pose = Odometry()
        self.bot_theta = 0
        
        # Create a Transform Broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

    def callback(self, msg: PoseStamped):
        global k, initial_x, initial_y
        
        # On the first callback, store the initial position
        if k == 0:
            initial_x = msg.pose.position.x
            initial_y = msg.pose.position.y
            k += 1
        else:
            # Update the position based on the initial position
            self.Pose.pose.pose.position.x = msg.pose.position.x - initial_x
            self.Pose.pose.pose.position.y = msg.pose.position.y - initial_y
            
            # Extract orientation quaternion and convert to Euler angles
            orientation_q = msg.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            r = R.from_quat(quaternion)
            euler = r.as_euler('xyz', degrees=False)
            self.bot_theta = euler[2]
            
            # Update the orientation in the Pose message
            self.Pose.pose.pose.orientation.x = orientation_q.x
            self.Pose.pose.pose.orientation.y = orientation_q.y
            self.Pose.pose.pose.orientation.z = orientation_q.z
            self.Ppose.pose.pose.orientation.w = orientation_q.w

            # Log the position and yaw angle
            self.get_logger().info(f'X: {self.Pose.pose.pose.position.x}, Y: {self.Pose.pose.pose.position.y}, yaw: {round(self.bot_theta, 2)}')
            
            # Publish the Odometry message
            self.pub.publish(self.Pose)
            
            # Publish the transform
            self.publish_transform()

    def publish_transform(self):
        # Function to publish TF of 'odom' with respect to 'base_link'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set the translation
        t.transform.translation.x = self.Pose.pose.pose.position.x
        t.transform.translation.y = self.Pose.pose.pose.position.y
        t.transform.translation.z = 0.0
        
        # Set the rotation
        t.transform.rotation.x = self.Pose.pose.pose.orientation.x
        t.transform.rotation.y = self.Pose.pose.pose.orientation.y
        t.transform.rotation.z = self.Pose.pose.pose.orientation.z
        t.transform.rotation.w = self.Pose.pose.pose.orientation.w
        
        # Send the transform
        self.br.sendTransform(t)

def main():
    rclpy.init()
    pose_est = PoseEstimator()
    rclpy.spin(pose_est)
    pose_est.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
