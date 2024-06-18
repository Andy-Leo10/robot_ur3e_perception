from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='alt_yolov5',
            name='alt_yolov5_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='alt_transform',
            name='alt_transform_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='alt_marker',
            name='alt_marker_launch'
        ),
    ])