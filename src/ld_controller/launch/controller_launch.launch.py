import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
    [
        launch_ros.actions.Node(
            package='ld_controller',
            executable='publisher',
            name='controller_publisher',
            parameters=[

            ]
        ),
        launch_ros.actions.Node(
            package='ld_controller',
            executable='subscriber',
            name='controller_subscriber',
            parameters=[

            ]
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
    ]
    )