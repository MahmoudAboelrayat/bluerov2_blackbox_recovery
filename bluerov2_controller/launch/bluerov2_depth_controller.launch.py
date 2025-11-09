from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Locate the config file dynamically
    config = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'param',
        'controller_param.yaml'
    ])

    return LaunchDescription([
        LifecycleNode(
            package='bluerov2_controller',
            executable='bluerov2_depth_hold',
            name='depth_controller',
            namespace='bluerov2',
            output='screen',
            parameters=[config],
        )
    ])
