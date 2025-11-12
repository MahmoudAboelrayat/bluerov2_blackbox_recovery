from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Namespace parameter
    namespace = LaunchConfiguration('namespace', default='bluerov2')

    return LaunchDescription([
        # Declare namespace argument
        DeclareLaunchArgument(
            'namespace',
            default_value='bluerov2',
            description='Robot namespace'
        ),

        # Gimbal lifecycle node
        LifecycleNode(
            package='bluerov2_controller',  # replace with your package name
            executable='bluerov2_gimbal',   # replace with your executable name
            name='bluerov2_gimbal',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
        ),
    ])
