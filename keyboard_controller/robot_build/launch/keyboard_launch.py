from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# launch this file
def generate_launch_description():
    configPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'params.yaml' ))
    print(configPath)
    return LaunchDescription([
        Node(
            package='keyboard_controller',
            namespace='keyboard',
            executable='controller',
            name='controller',
            parameters = [configPath],
            prefix = 'xterm -e'
            
        ),
        Node(
            package='keyboard_controller',
            namespace='twist_node',
            executable='twistPub',
            parameters = [configPath],
            name='publisher'
            
        )
    ])