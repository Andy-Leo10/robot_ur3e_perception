from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='sim_data',
            name='sim_data_launch'
        ),
    ])