from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]  # Gazebo plugin listens on /diff_cont/cmd_vel_unstamped
        )
    ])
