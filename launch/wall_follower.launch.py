from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_solver_bot',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        )
    ])
