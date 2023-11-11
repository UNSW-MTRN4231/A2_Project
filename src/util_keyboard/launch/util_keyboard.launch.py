from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='util_keyboard',
            executable='util_keyboard',
            name='util_keyboard',
            output='screen',
        ),
    ])