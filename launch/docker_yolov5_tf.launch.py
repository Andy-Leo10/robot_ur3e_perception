from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='docker_yolov5',
            name='docker_yolov5_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='alt_transform',
            name='alt_transform_launch'
        ),
    ])