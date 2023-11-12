from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='high_level_control',
            executable='high_level_control',
            name='high_level_control',
            output='screen',
            emulate_tty=True
        )
    ])
