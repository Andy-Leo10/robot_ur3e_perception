from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='real_yolov5',
            name='real_yolov5_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='real_transform',
            name='real_transform_launch'
        ),
    ])