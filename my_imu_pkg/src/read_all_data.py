#!/usr/bin/env python3
class BMX160:
    def __init__(self, bus=1):
        # Initialize I2C communication and sensor configuration
        pass

    def begin(self):
        # Sensor initialization and configuration
        pass

    def get_all_data(self):
        # Read all sensor data
        accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
        mag_x, mag_y, mag_z = 0.0, 0.0, 0.0

        # Return a tuple of all sensor data
        return (mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
