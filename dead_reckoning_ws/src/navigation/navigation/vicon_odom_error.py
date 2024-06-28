import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped, Pose2D
import pandas as pd
from scipy.interpolate import interp1d

def read_bag(bag_path, topic_name, message_type):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    messages = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data,message_type)
            messages.append((msg, t/1e9))

    return messages

def extract_xy_vicon(messages):
    x_shift = -0.346
    y_shift = 2.84
    xy_data = []
    for msg, timestamp in messages:
        x = msg.pose.position.x + x_shift
        y = msg.pose.position.y + y_shift  # Shift y by +2.84 cm
        xy_data.append((x, y, timestamp))
    return xy_data

def extract_xy_odom(messages):
    xy_data = []
    for msg, timestamp in messages:
        x = msg.x
        y = msg.y
        xy_data.append((x, y, timestamp))
    return xy_data

def plot_data(vicon_data, odom_data, title='Position Data'):
    vicon_x = [x for x, _, _ in vicon_data]
    vicon_y = [y for _, y, _ in vicon_data]
    odom_x = [x for x, _, _ in odom_data]
    odom_y = [y for _, y, _ in odom_data]

    plt.figure(figsize=(10, 6))
    plt.plot(vicon_x, vicon_y, label='Vicon Data (Shifted)')
    plt.plot(odom_x, odom_y, label='Odom Data')
    plt.title(title)
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    bag_path_odom = '/home/tahsin-khan/odom_data/rosbag2_2024_06_27-15_05_35_0.mcap'
    bag_path_vicon = '/home/tahsin-khan/vicon_data/rosbag2_2024_06_27-15_05_24_0.mcap'
    topic_name_vicon = '/vicon/RCCar/RCCar/pose'
    topic_name_odom = '/odom'

    vicon_messages = read_bag(bag_path_vicon, topic_name_vicon, PoseStamped)
    odom_messages = read_bag(bag_path_odom, topic_name_odom, Pose2D)

    vicon_xy = extract_xy_vicon(vicon_messages)
    odom_xy = extract_xy_odom(odom_messages)

    if not vicon_xy or not odom_xy:
        print("No data to process.")
        return

    # Plot original data with the y-shift applied to Vicon data
    plot_data(vicon_xy, odom_xy, title='Vicon and Odom Data with Vicon y-shifted by +2.84 cm')

if __name__ == '__main__':
    main()