from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Locate the config.yaml file
    config_path = '/home/toaster/ros2_ws/src/vn_keyboard_controller/config/config.yaml'

    # Ask the user if they want to load the keyboard node
    load_keyboard = input("Do you want to load the keyboard node? (y/n): ").strip().lower()

    # Create a list of nodes to launch
    nodes = [
        Node(
            package='vn_keyboard_controller',
            executable='twist_node',  # Use the executable name
            name='twist_node',
            parameters=[config_path]  # Load the config file here
        )
    ]

    # Only add the keyboard node if the user agreed
    if load_keyboard == 'y':
        nodes.append(
            Node(
                package='vn_keyboard_controller',
                executable='keyboard_controller',  # Use the executable name
                name='keyboard_controller',
                parameters=[config_path]  # Load the config file here
            )
        )
    
    return LaunchDescription(nodes)
