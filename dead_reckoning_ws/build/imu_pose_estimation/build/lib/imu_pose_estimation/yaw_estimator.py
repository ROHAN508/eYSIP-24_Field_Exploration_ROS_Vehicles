import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import time
import math

class YawEstimator(Node):
    def __init__(self):
        super().__init__('yaw_estimator_node')
        
        # Subscribers
        self.sub_imu = self.create_subscription(Imu, '/imu/raw', self.imu_callback, 10)
        self.sub_mag = self.create_subscription(Float64MultiArray, '/magno', self.mag_callback, 10)
        
        # Publisher
        self.pub_yaw = self.create_publisher(Float64, '/yaw', 10)
        
        # Kalman filter variables
        self.yaw = 0.0  # Initialize yaw to zero
        self.yaw_rate = 0.0
        self.mag_yaw = 0.0
        self.last_time = time.time()
        
        # Kalman filter parameters
        self.yaw_estimate = 0.0
        self.P = 1.0
        self.Q = 0.01  # Process noise covariance
        self.R = 0.03  # Measurement noise covariance
        self.K = 0.0
        self.bias_list = []

        self.IMU_YAW_OFFSET = 0.0

        self.initial_yaw_set = False
        self.initial_yaw = 0.0

        # Roll and pitch from accelerometer
        self.roll = 0.0
        self.pitch = 0.0
        
    def imu_callback(self, msg: Imu):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Ensure dt is positive and not too small
        if dt <= 0 or dt > 1:
            return
        
        if len(self.bias_list) < 50:
            self.bias_list.append(msg.angular_velocity.z)
            if len(self.bias_list) == 50:
                self.IMU_YAW_OFFSET = np.mean(self.bias_list)
                self.get_logger().info(f'IMU bias calculated: {self.IMU_YAW_OFFSET}')
        else:
            self.yaw_rate = msg.angular_velocity.z - self.IMU_YAW_OFFSET

            # Extract roll and pitch from accelerometer data
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            self.roll = math.atan2(-ax, math.sqrt((ay*ay) + (az*az)))
            self.pitch = math.atan2(ay, math.sqrt((ax * ax) + (az * az)))

            # pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
#    roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

            # Predict step
            self.yaw_estimate += self.yaw_rate * dt
            self.P += self.Q
        
            # Update step
            y = self.mag_yaw - self.yaw_estimate
            self.K = self.P / (self.P + self.R)
            self.yaw_estimate += self.K * y
            self.P *= (1 - self.K)
        
            self.yaw = self.yaw_estimate
        
            # Publish the yaw value
            yaw_msg = Float64()
            yaw_msg.data = self.yaw
            self.pub_yaw.publish(yaw_msg)
            self.get_logger().info(f'Yaw: {self.yaw:.2f} radians, Yaw Rate: {self.yaw_rate:.2f} rad/s, dt: {dt:.4f}s')
        
    def mag_callback(self, msg: Float64MultiArray):
        # Compute yaw from magnetometer data (units are microteslas)
        mag_x = msg.data[0] * 1e-6
        mag_y = -msg.data[1] * 1e-6  # Negate y component
        mag_z = -msg.data[2] * 1e-6  # Negate z component
        
        # Tilt compensation
        sin_roll = math.sin(self.roll)
        cos_roll = math.cos(self.roll)
        sin_pitch = math.sin(self.pitch)
        cos_pitch = math.cos(self.pitch)
        
        mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch *cos_roll +mag_y*sin_roll*sin_pitch
        mag_y_comp = mag_y * cos_roll - mag_z*sin_roll

        # loat Yh = (magY * cos(roll)) - (magZ * sin(roll));
#    float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch))
        
        # Calculate magnetometer yaw
        self.mag_yaw = math.atan2(mag_y_comp, mag_x_comp)
        
        if not self.initial_yaw_set:
            self.initial_yaw = self.mag_yaw
            self.initial_yaw_set = True
        
        # Normalize the yaw to be within the range of -pi to pi
        self.mag_yaw = (self.mag_yaw - self.initial_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Log magnetometer yaw for debugging
        self.get_logger().info(f'Magnetometer yaw: {self.mag_yaw:.2f} radians')
        
def main(args=None):
    rclpy.init(args=args)
    node = YawEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
