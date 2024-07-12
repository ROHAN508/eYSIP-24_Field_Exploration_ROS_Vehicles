#!/usr/bin/env python

#################################################################################
#   Copyright Lars Ludvigsen. All Rights Reserved.                              #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
imu_node.py
"""

import sys
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from imu_pkg import DFRobot_BMX160
from imu_pkg import constants
import numpy as np

class IMUNode(Node):
    """Node responsible for collecting IMU data and publishing it."""

    def __init__(self):
        """Create an IMUNode."""
        super().__init__("imu_node")
        self.get_logger().info("imu_node started.")
        self.stop_queue = threading.Event()
        
        # Declare and get parameters
        self.imu_i2c_bus_id = self.declare_parameter("~bus_id", 1).value
        self.sensor = DFRobot_BMX160.BMX160(self.imu_i2c_bus_id)
        self.sensor.soft_reset()
     
        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", '/imu/raw').value
        self.imu_i2c_address = self.declare_parameter("~device_address", 104).value
        self.publish_rate = self.declare_parameter("~publish_rate", 250).value
        self.get_logger().info(f"imu_frame: {self.imu_frame}, pub_topic: {self.pub_topic}, publish_rate: {self.publish_rate}")

        # Publishers
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu, self.pub_topic, 1, callback_group=self.imu_message_pub_cb_grp)
        self.magno_message_publisher = self.create_publisher(Float64MultiArray, '/magno', 1, callback_group=self.pub_cb_grp)

        # Initialize biases and scales
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.magno_bias = np.zeros(3)
        self.magno_scale = np.ones(3)
        self.calibrated = False

        # Heartbeat timer
        self.timer_count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"IMU node successfully created. Publishing on topic: {self.pub_topic}")

    def timer_callback(self):
        """Heartbeat function to keep the node alive."""
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement."""
        try:
            while not self.sensor.begin():
                self.get_logger().info("Sensor initialization failed. Retrying...")
                time.sleep(2)

            # Set sensor ranges
            self.sensor.set_gyro_range(0)
            self.sensor.set_magn_conf()
            self.sensor.set_accel_range(0)
        except Exception as ex:
            self.get_logger().info(f"Failed to create IMU monitor: {ex}")
            raise ex

        self.get_logger().info('Initialization and calibration of IMU sensor done.')

        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Called when the object is destroyed."""
        self.get_logger().info('Exiting.')
        self.stop_queue.set()
        self.rate.destroy()
        self.thread.join()

    def processor(self):
        """Processor that publishes IMU messages at a specified rate."""
        self.get_logger().info(f"Publishing messages at {self.publish_rate} Hz.")
        self.rate = self.create_rate(self.publish_rate)

        while not self.stop_queue.is_set() and rclpy.ok():
            try:
                self.publish_imu_message()
                self.rate.sleep()
            except Exception as ex:
                self.get_logger().error(f"Failed to create IMU message: {ex}")

    # def calibrate_imu(self):
    #     """Calibrate the IMU sensors."""
    #     self.get_logger().info("Starting IMU calibration...")
        
    #     # Static calibration for accelerometer and gyroscope
    #     self.calibrate_accel_gyro()
        
    #     # Calibration for magnetometer
    #     self.calibrate_magno()
        
    #     self.calibrated = True
    #     self.get_logger().info("IMU calibration completed.")

    # def calibrate_accel_gyro(self):
    #     """Calibrate accelerometer and gyroscope."""
    #     self.get_logger().info("Calibrating accelerometer and gyroscope...")
    #     accel_samples = []
    #     gyro_samples = []

    #     for _ in range(100):
    #         data = self.sensor.get_all_data()
    #         accel_samples.append(data[6:9])
    #         gyro_samples.append(data[3:6])
    #         time.sleep(0.1)

    #     self.accel_bias = np.mean(accel_samples, axis=0)
    #     self.gyro_bias = np.mean(gyro_samples, axis=0)

    #     self.get_logger().info(f"Accelerometer bias: {self.accel_bias}")
    #     self.get_logger().info(f"Gyroscope bias: {self.gyro_bias}")

    # def calibrate_magno(self):
    #     """Calibrate magnetometer."""
    #     self.get_logger().info("Calibrating magnetometer...")
    #     magno_samples = []

    #     for _ in range(200):
    #         data = self.sensor.get_all_data()
    #         magno_samples.append(data[0:3])
    #         time.sleep(0.1)

    #     magno_samples = np.array(magno_samples)
    #     magno_mean = np.mean(magno_samples, axis=0)
    #     magno_cov = np.cov(magno_samples, rowvar=False)
    #     eigvals, eigvecs = np.linalg.eigh(magno_cov)
    #     self.magno_scale = 1.0 / np.sqrt(eigvals)
    #     self.magno_bias = magno_mean

    #     self.get_logger().info(f"Magnetometer bias: {self.magno_bias}")
    #     self.get_logger().info(f"Magnetometer scale: {self.magno_scale}")

    def publish_imu_message(self):
        """Publish the sensor message when we get new data."""
        try:
            imu_msg = Imu()
            data = self.sensor.get_all_data()

            # Gyroscope data in rad/s
            gyro = Vector3()
            gyro.x = data[3] * (math.pi / 180) * constants.GRAVITY_CONSTANT * 1.58
            gyro.y = data[4] * (math.pi / 180) * constants.GRAVITY_CONSTANT * 1.58
            gyro.z = data[5] * (math.pi / 180) * constants.GRAVITY_CONSTANT * 1.6
            
            # Accelerometer data in m/s²
            accel = Vector3()
            accel.x = data[6]
            accel.y = data[7]
            accel.z = data[8]

            # Magnetometer data
            magno_msg = Float64MultiArray()
            magno_x = data[0]
            magno_y = data[1]
            magno_z = data[2]
            magno_msg.data = np.array([magno_x, magno_y, magno_z])

            # Fill IMU message
            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.EMPTY_ARRAY_9
            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0
            
            # Add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame

            self.get_logger().debug(f'Gyro Z: {gyro.z:.2f}')

            # Publish messages
            self.imu_message_publisher.publish(imu_msg)
            self.magno_message_publisher.publish(magno_msg)
        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")
def main(args=None):
    try:
        rclpy.init(args=args)
        with IMUNode() as imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(imu_node, executor)
        imu_node.destroy_node()
    except KeyboardInterrupt:
        pass    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
