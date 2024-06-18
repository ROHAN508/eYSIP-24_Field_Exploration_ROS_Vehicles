from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_imu_pkg',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='my_imu_pkg',
            executable='imu_fusion_node',
            name='imu_fusion_node',
            output='screen'
        ),
        Node(
            package='my_imu_pkg',
            executable='imu_tf_node',
            name='imu_tf_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/rviz_config.rviz'],
            output='screen'
        )
    ])
