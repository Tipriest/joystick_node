from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_node',
            executable='joystick_node',
            name='joystick_node',
            output='screen',
            parameters=[{
                'joystick_type': 'ps5',
                'joystick_device': '/dev/input/js0',
                'joystick_bits': 16
            }]
        )
    ])
