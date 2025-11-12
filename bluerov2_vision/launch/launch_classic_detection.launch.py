from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace('bluerov2'),

        # Start classic detection node
        Node(
            package='bluerov2_vision',
            executable='classic_detection',
            name='classic_detection_node',
            output='screen'
        ),
    ])