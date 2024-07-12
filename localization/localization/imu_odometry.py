import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

# Initialize global variables for IMU offsets and other counters
IMU_ACC_OFFSET = 0.0
IMU_YAW_OFFSET = 0.0
i = 0
j = 0

class PoseEstimator(Node):
    def __init__(self):
        # Initialize the node with the name 'Dead_reckoning_node'
        super().__init__('Dead_reckoning_node')

        # Subscription to PWM values
        self.sub_pwm = self.create_subscription(Int32MultiArray, '/pwm_val', self.pwm_callback, 10)

        # Subscription to IMU data with a specified QoS profile
        self.sub_imu = self.create_subscription(Imu, '/imu/raw', self.callBack, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10))

        # Broadcaster for transform messages
        self.br = tf2_ros.TransformBroadcaster(self)

        # Publisher for Odometry messages
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        # Initialize Odometry message
        self.Pose = Odometry()
        self.Pose.header.frame_id = 'odom'
        self.Pose.child_frame_id = 'base_link'

        # Initialize bias lists for IMU calibration
        self.bias_list = np.zeros(50)
        self.bias_yaw_list = np.zeros(50)

        # Initialize other variables for pose estimation
        self.pwm = 0.0
        self.bias = 0.0
        self.pwm_val = np.array([-30, -25, -20, 0, 20, 25, 30, 35, 40])
        self.speed = np.array([-0.707, -0.545, -0.36, 0, 0.30, 0.4, 0.68, 0.89, 1.06])
        self.x = np.array([[0.0], [0.0]])
        self.P = np.array([[1, 0.0], [0.0, 1]])
        self.Q = np.array([[0.1, 0.0], [0.0, 0.3]])
        self.R = np.array([[0.05, 0.0], [0.0, 0.05]])
        self.H = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.I = np.eye(2)
        self.last_time = time.time()
        self.dt = 0.0
        self.acc_x = 0.0
        self.z = 0
        self.yaw = 0.0
        self.prev_dist = 0.0
        self.vector_length = 0.0
        self.pwm_throttle = 0
        self.pwm_servo = 1800
        
    def pwm_callback(self, msg1: Int32MultiArray):
        # Update PWM values from the message
        self.pwm = msg1.data
        self.pwm_throttle = self.pwm[0]
        self.pwm_servo = self.pwm[1]

    def callBack(self, msg: Imu):
        global i, j, IMU_ACC_OFFSET, IMU_YAW_OFFSET

        # Extract linear acceleration and angular velocity from the IMU message
        self.linear_acc = msg.linear_acceleration
        self.angular_vel = msg.angular_velocity

        # Calculate time difference
        self.current_time = time.time()
        self.dt = self.current_time - self.last_time
        self.last_time = time.time()

        # Calibrate IMU yaw offset
        if j < 50:
            self.bias_yaw_list[j] = msg.angular_velocity.z
            self.get_logger().info(f'bias: {msg.angular_velocity.z}')
            j += 1
            if j == 49:
                IMU_YAW_OFFSET = np.sum(self.bias_yaw_list) / 50
                self.get_logger().info(f'IMU bias calculated: {IMU_YAW_OFFSET}')
        else:
            # Update yaw based on angular velocity and offset
            self.yaw = self.yaw + (self.angular_vel.z - IMU_YAW_OFFSET) * self.dt

            # Handle zero throttle case
            if self.pwm_throttle == 0.0:
                self.x[1, 0] = 0.0
                self.acc_x = 0.0
                if i < 50:
                    self.bias_list[i] = self.linear_acc.x
                    IMU_ACC_OFFSET = np.sum(self.bias_list) / 50
                    i += 1
                else:
                    i = 0
            else:
                # Update state estimates using Kalman filter equations
                self.acc_x = self.linear_acc.x - IMU_ACC_OFFSET
                self.F = np.array([[1, self.dt], [0, 1]])
                self.B = np.array([[0.5 * self.dt**2], [self.dt]])
                self.u = np.array([[self.acc_x]])
                self.x_pred = self.F @ self.x + self.B @ self.u
                self.P_pred = self.F @ self.P @ self.F.T + self.Q
                self.z = np.array([[self.x[0, 0] + np.interp(self.pwm_throttle, self.pwm_val, self.speed) * self.dt], [np.interp(self.pwm_throttle, self.pwm_val, self.speed)]])
                self.y = self.z - (self.H @ self.x_pred)
                self.S = self.H @ self.P_pred @ self.H.T + self.R
                self.K = self.P_pred @ self.H.T @ np.linalg.inv(self.S)
                self.x = self.x_pred + self.K @ self.y
                self.P = (self.I - self.K @ self.H) @ self.P_pred
                self.vector_length = self.x[0, 0] - self.prev_dist
                self.prev_dist = self.x[0, 0]

                # Update pose estimates
                self.Pose.header.stamp = self.get_clock().now().to_msg()
                self.Pose.pose.pose.position.x += self.vector_length * math.cos(self.yaw)
                self.Pose.pose.pose.position.y += self.vector_length * math.sin(self.yaw)

            # Normalize yaw angle
            self.yaw_norm = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            q = R.from_euler('z', self.yaw_norm).as_quat()
            self.Pose.pose.pose.orientation.x = q[0]
            self.Pose.pose.pose.orientation.y = q[1]
            self.Pose.pose.pose.orientation.z = q[2]
            self.Pose.pose.pose.orientation.w = q[3]

            # Log pose information
            self.get_logger().info(f'X: {self.Pose.pose.pose.position.x}, Y: {self.Pose.pose.pose.position.y }, yaw: {round(self.yaw_norm, 2)}')
            self.get_logger().info(f'acc off: {IMU_ACC_OFFSET}')

            # Publish pose and transform
            self.pub.publish(self.Pose)
            self.publish_transform()

    def publish_transform(self):
        # Function to publish transform of 'odom' w.r.t to 'base_link'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.Pose.pose.pose.position.x
        t.transform.translation.y = self.Pose.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.Pose.pose.pose.orientation.x
        t.transform.rotation.y = self.Pose.pose.pose.orientation.y
        t.transform.rotation.z = self.Pose.pose.pose.orientation.z
        t.transform.rotation.w = self.Pose.pose.pose.orientation.w
        self.br.sendTransform(t)

def main():
    # Initialize ROS
    rclpy.init()
    
    # Create PoseEstimator node
    pose_est = PoseEstimator()
    
    # Keep the node running
    rclpy.spin(pose_est)
    
    # Clean up
    pose_est.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
