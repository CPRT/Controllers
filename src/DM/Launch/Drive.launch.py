from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='DM',
            executable='Controller',
            name='Controller'
        ),
        Node(
            package='DM',
            executable='test',
            name='test'
        ),
    ])