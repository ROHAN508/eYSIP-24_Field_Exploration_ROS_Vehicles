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
from geometry_msgs.msg import Vector3
from imu_pkg import DFRobot_BMX160
from imu_pkg import constants

class IMUNode(Node):
    """Node responsible for collecting IMU data and publishing it."""
    
    def __init__(self):
        """Create an IMUNode."""
        super().__init__("imu_node")
        self.get_logger().info("imu_node started.")
        self.stop_queue = threading.Event()
        
        self.imu_i2c_bus_id = self.declare_parameter("~bus_id", 1).value
        self.sensor = DFRobot_BMX160.BMX160(self.imu_i2c_bus_id)
        self.sensor.soft_reset()
     
        # Initialize parameters
        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", '/imu_msg/raw').value
        self.imu_i2c_address = self.declare_parameter("~device_address", 104).value
        self.publish_rate = self.declare_parameter("~publish_rate", 125).value
        self.get_logger().info(f"imu_frame: {self.imu_frame}, pub_topic: {self.pub_topic}, publish_rate: {self.publish_rate}")

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu, self.pub_topic, 1, callback_group=self.imu_message_pub_cb_grp)

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

    def publish_imu_message(self):
        """Publish the sensor message when we get new data."""
        try:
            imu_msg = Imu()
            data = self.sensor.get_all_data()
            

            # Gyroscope data in rad/s
            gyro = Vector3()
            gyro.x = data[3] * (math.pi / 180) * 9.8
            gyro.y = data[4] * (math.pi / 180) *9.8
            gyro.z = data[5] * (math.pi / 180) *9.8
            
            # Accelerometer data in m/s²
            accel = Vector3()
            accel.x = data[6]
            accel.y = data[7]
            accel.z = data[8]

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

            self.imu_message_publisher.publish(imu_msg)
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
