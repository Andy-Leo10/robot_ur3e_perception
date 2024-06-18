from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='sim_dip',
            name='sim_dip_launch'
        ),
        Node(
            package='robot_ur3e_perception',
            executable='sim_transform',
            name='sim_transform_launch'
        ),
    ])