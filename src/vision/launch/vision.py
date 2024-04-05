from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',  # package name
            executable='track_vision',  #  the name of node/executable
            name='track_vision_node',  # name for the launched node
            output='screen',
            parameters=[{
                # List any ROS parameters here
            }],
            arguments=[
                # List any arguments your node needs here
            ],
        ),
    ])

