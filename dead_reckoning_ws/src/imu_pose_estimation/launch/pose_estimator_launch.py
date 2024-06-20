import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = get_package_share_directory('imu_pose_estimation')
    urdf_path = os.path.join(pkg_share, 'description', 'car.urdf.xacro')
    # rviz_config_path = os.path.join(pkg_share, 'rviz', 'imu_visual.rviz')

    robot_description = Command(['xacro ', urdf_path])

    # Node to publish robot state
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': robot_description, }],
    arguments=[urdf_path])

    # Node to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_path],
    )

    # Pose estimator node
    pose_estimator_node = Node(
        package='imu_pose_estimation',  # Replace with your package name
        executable='pose_estimator_node',  # Replace with your node executable name
        name='pose_estimator',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        # pose_estimator_node,
    ])
