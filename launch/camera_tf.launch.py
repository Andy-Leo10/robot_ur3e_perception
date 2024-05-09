from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='camera',
            name='camera_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='transform',
            name='transform_launch'
        ),
    ])