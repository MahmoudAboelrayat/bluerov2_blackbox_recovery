from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Locate the config file dynamically
    namespace = LaunchConfiguration('namespace', default='bluerov2')

    config = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'param',
        'controller_param.yaml'
    ])

    return LaunchDescription([
        LifecycleNode(
            package='bluerov2_controller',
            executable='bluerov2_yaw_hold',
            name='yaw_controller',
            namespace=namespace,
            output='screen',
            parameters=[config],
        )
    ])
