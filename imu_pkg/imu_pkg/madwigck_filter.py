# -*- coding: utf-8 -*-
"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import warnings
import numpy as np
from numpy.linalg import norm
from .quaternion import Quaternion  # Ensure quaternion.py is available and correct

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class MadgwickAHRS(Node):
    samplePeriod = 1 / 256
    quaternion = Quaternion(1, 0, 0, 0)
    beta = 1
    zeta = 0

    def __init__(self, sampleperiod=None, quaternion=None, beta=None, zeta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :param zeta: Algorithm gain zeta
        :return:
        """
        super().__init__("imu_filter_node")
        self.get_logger().info("imu_filter_node started.")

        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if zeta is not None:
            self.zeta = zeta

        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.sub_topic = self.declare_parameter("~imu_sub_topic", '/imu_msg/raw').value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", '/orientation').value
        self.publish_rate = self.declare_parameter("~publish_rate", 250).value

        self.get_logger().info(f"imu_frame: {self.imu_frame}, sub_topic: {self.sub_topic}, pub_topic: {self.pub_topic}, publish_rate: {self.publish_rate}")

        self.imu_message_sub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()

        self.imu_message_subscriber = self.create_subscription(Imu, self.sub_topic, self.imu_callback, 1, callback_group=self.imu_message_sub_cb_grp , qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)) 
        self.imu_message_publisher = self.create_publisher(Imu, self.pub_topic, 1, callback_group=self.imu_message_pub_cb_grp)

        self.get_logger().info(f"IMU filter node successfully created. Subscribed to topic: {self.sub_topic}")

    def update(self, gyroscope, accelerometer, magnetometer):
        """
        Perform one update step with data from an AHRS sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        :param magnetometer: A three-element array containing the magnetometer data. Can be any unit since a normalized value is used.
        :return:
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()
        magnetometer = np.array(magnetometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Normalise magnetometer measurement
        if norm(magnetometer) == 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)

        h = q * (Quaternion(0, magnetometer[0], magnetometer[1], magnetometer[2]) * q.conj())
        b = np.array([0, norm(h[1:3]), 0, h[3]])

        # Gradient descent algorithm corrective step
        f = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]) - accelerometer[0],
            2 * (q[0] * q[1] + q[2] * q[3]) - accelerometer[1],
            2 * (0.5 - q[1] ** 2 - q[2] ** 2) - accelerometer[2],
            2 * b[1] * (0.5 - q[2] ** 2 - q[3] ** 2) + 2 * b[3] * (q[1] * q[3] - q[0] * q[2]) - magnetometer[0],
            2 * b[1] * (q[1] * q[2] - q[0] * q[3]) + 2 * b[3] * (q[0] * q[1] + q[2] * q[3]) - magnetometer[1],
            2 * b[1] * (q[0] * q[2] + q[1] * q[3]) + 2 * b[3] * (0.5 - q[1] ** 2 - q[2] ** 2) - magnetometer[2]
        ])
        j = np.array([
            [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
            [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
            [0, -4 * q[1], -4 * q[2], 0],
            [-2 * b[3] * q[2], 2 * b[3] * q[3], -4 * b[1] * q[2] - 2 * b[3] * q[0], -4 * b[1] * q[3] + 2 * b[3] * q[1]],
            [-2 * b[1] * q[3] + 2 * b[3] * q[1], 2 * b[1] * q[2] + 2 * b[3] * q[0], 2 * b[1] * q[1] + 2 * b[3] * q[3], -2 * b[1] * q[0] + 2 * b[3] * q[2]],
            [2 * b[1] * q[2], 2 * b[1] * q[3] - 4 * b[3] * q[1], 2 * b[1] * q[0] - 4 * b[3] * q[2], 2 * b[1] * q[1]]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Gyroscope compensation drift
        gyroscopeQuat = Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])
        stepQuat = Quaternion(step.T[0], step.T[1], step.T[2], step.T[3])

        gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * self.samplePeriod * self.zeta * -1

        # Compute rate of change of quaternion
        qdot = (q * gyroscopeQuat) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion

    def update_imu(self, gyroscope, accelerometer):
        """
        Perform one update step with data from an IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        f = np.array([
            2 * (q[1] * q[3] - q[0] * q[2]) - accelerometer[0],
            2 * (q[0] * q[1] + q[2] * q[3]) - accelerometer[1],
            2 * (0.5 - q[1] ** 2 - q[2] ** 2) - accelerometer[2]
        ])
        j = np.array([
            [-2 * q[2], 2 * q[3], -2 * q[0], 2 * q[1]],
            [2 * q[1], 2 * q[0], 2 * q[3], 2 * q[2]],
            [0, -4 * q[1], -4 * q[2], 0]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Compute rate of change of quaternion
        qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion

    def imu_callback(self, msg):
        """Callback function to process incoming IMU data."""
        # Extract angular velocity (gyroscope) and linear acceleration (accelerometer) data
        gyroscope = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        accelerometer = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        # Update the filter use the ahrs function for magnetometer reading 
        self.update_imu(gyroscope, accelerometer)

        # Create a new Imu message with the filtered data
        filtered_msg = Imu()
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.header.frame_id = self.imu_frame

        # Set angular velocity (gyroscope) to zero as it's not used in the output
        filtered_msg.angular_velocity = Vector3()
        filtered_msg.angular_velocity_covariance = [0.0] * 9

        # Set linear acceleration to zero as it's not used in the output
        filtered_msg.linear_acceleration = Vector3()
        filtered_msg.linear_acceleration_covariance = [0.0] * 9

        # Set the orientation from the filter's quaternion
        q = self.quaternion
        filtered_msg.orientation.x = q[1]
        filtered_msg.orientation.y = q[2]
        filtered_msg.orientation.z = q[3]
        filtered_msg.orientation.w = q[0]
        filtered_msg.orientation_covariance = [0.0] * 9

        # Publish the filtered IMU message
        self.imu_message_publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MadgwickAHRS()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
