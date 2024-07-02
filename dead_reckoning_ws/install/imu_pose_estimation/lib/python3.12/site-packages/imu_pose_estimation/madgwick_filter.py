import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import math
from std_msgs.msg import Float64MultiArray

class MadgwickFilter:
    def __init__(self, beta=0.14, sample_freq=100.0):
        self.beta = beta
        self.sample_freq = sample_freq
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion representing the orientation

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        q1, q2, q3, q4 = self.q
        beta = self.beta
        sample_freq = self.sample_freq

        # Normalize accelerometer measurement
        if ax != 0.0 or ay != 0.0 or az != 0.0:
            recip_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

        # Normalize magnetometer measurement
        if mx != 0.0 or my != 0.0 or mz != 0.0:
            recip_norm = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

        # Reference direction of Earth's magnetic field
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1mx = _2q1 * mx
        _2q1my = _2q1 * my
        _2q1mz = _2q1 * mz
        _2q2mx = _2q2 * mx

        hx = mx * q1 * q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2 * q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3 * q3 - mx * q4 * q4
        hy = _2q1mx * q4 + my * q1 * q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2 * q2 + my * q3 * q3 + _2q3 * mz * q4 - my * q4 * q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1 * q1 + _2q2mx * q4 - mz * q2 * q2 + _2q3 * my * q4 - mz * q3 * q3 + mz * q4 * q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q2 * (2.0 * q1 * q2 + _2q3 * q4 - ay)
        s2 = _2q4 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q1 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az)
        s3 = -_2q1 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q4 * (2.0 * q1 * q2 + _2q3 * q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az)
        s4 = _2q2 * (2.0 * q2 * q4 - _2q1 * q3 - ax) + _2q3 * (2.0 * q1 * q2 + _2q3 * q4 - ay)

        recip_norm = 1.0 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 *= recip_norm
        s2 *= recip_norm
        s3 *= recip_norm
        s4 *= recip_norm

        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

        q1 += q_dot1 * (1.0 / sample_freq)
        q2 += q_dot2 * (1.0 / sample_freq)
        q3 += q_dot3 * (1.0 / sample_freq)
        q4 += q_dot4 * (1.0 / sample_freq)

        recip_norm = 1.0 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = np.array([q1, q2, q3, q4]) * recip_norm

    def get_yaw(self):
        q1, q2, q3, q4 = self.q
        return math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))


class ImuYawEstimatorNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_estimator_node')
        self.mag_sub = self.create_subscription(Float64MultiArray, '/magno', self.magno_callback, 10)
        self.subscription = self.create_subscription(Imu, '/imu/raw', self.imu_callback, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10))
        self.subscription  # prevent unused variable warning
        self.madgwick_filter = MadgwickFilter()
        self.initial_yaw_set = False
        self.initial_yaw = 0.0
        self.get_logger().info('IMU Yaw Estimator Node Initialized')
        self.mx = 0.0  # Replace with actual magnetometer data
        self.my = 0.0  # Replace with actual magnetometer data
        self.mz = 0.0  # Replace with actual magnetometer data

    def magno_callback(self, msg: Float64MultiArray):
        self.mag = msg.data
        self.mx, self.my, self.mz = self.mag[0]*(math.pow(10,-6)),self.mag[1]*(math.pow(10,-6)),self.mag[2]*(math.pow(10,-6))

    def imu_callback(self, msg):
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        self.get_logger().info(f'mx {self.mx} , my {self.my} , mz {self.mz}')

        self.madgwick_filter.update(gx, gy, gz, ax, ay, az, self.mx, self.my, self.mz)
        current_yaw = self.madgwick_filter.get_yaw()

        if not self.initial_yaw_set:
            self.initial_yaw = current_yaw
            self.initial_yaw_set = True

        yaw = current_yaw - self.initial_yaw
        self.get_logger().info(f'Yaw: {yaw:.2f} radians')

def main(args=None):
    rclpy.init(args=args)
    imu_yaw_estimator_node = ImuYawEstimatorNode()
    rclpy.spin(imu_yaw_estimator_node)
    imu_yaw_estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
