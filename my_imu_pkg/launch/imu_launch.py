import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_imu_pkg',
            executable='imu_fusion_node.py',  # Note the .py extension
            name='imu_fusion_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='my_imu_pkg',
            executable='imu_node.py',  # Note the .py extension
            name='imu_node',
            output='screen'
        )
    ])
