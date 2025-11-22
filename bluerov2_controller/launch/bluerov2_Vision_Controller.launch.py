from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='bluerov2')

    return LaunchDescription([
        Node(
            package='bluerov2_controller',
            executable='bluerov2_Vision_Controller',
            name='bluerov2_Vision_Controller',
            namespace=namespace,
            parameters=[{}]
            # remappings=[('old_topic', 'new_topic')]
        )
    ])
