from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'world': 'task_6'}.items()
        ),
        Node(
            package='task_6',
            executable='red_ball_tracker',
            name='red_ball_tracker'
        )
    ])
