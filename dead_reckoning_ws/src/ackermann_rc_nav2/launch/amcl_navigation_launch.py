from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    ackermann_rc_nav2_dir = FindPackageShare('ackermann_rc_nav2')

    map_file = PathJoinSubstitution([ackermann_rc_nav2_dir, 'map', 'map.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='Whether to set map_subscribe_transient_local'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'localization_launch.py'])
            ]),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_subscribe_transient_local': LaunchConfiguration('map_subscribe_transient_local')
            }.items(),
        ),
    ])
