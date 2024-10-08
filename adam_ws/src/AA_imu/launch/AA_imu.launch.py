import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


config = os.path.join(get_package_share_directory('AA_imu'), 
    'config', 
    'imu_params.yaml')
  

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='AA_imu',
            executable='imu_node',
            name='imu_node',
            parameters = [config]
        ),
        Node(
            package='AA_imu',
            executable='imu_twist',
            name='imu_twist',
            parameters = [config]
        )
    ])
