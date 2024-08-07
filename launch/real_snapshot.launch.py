from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_ur3e_perception',
            executable='snapshot',
            name='real_snapshot_launch',
            parameters=[{'use_sim_time': False}]
        ),
    ])