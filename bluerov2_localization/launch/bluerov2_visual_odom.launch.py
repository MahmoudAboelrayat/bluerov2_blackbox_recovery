import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Robot namespace'
    )

    pkg_path = get_package_share_directory('bluerov2_localization')
    param_file_path = os.path.join(pkg_path, 'param', 'camera_calibration_11_11.npz')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='bluerov2_localization',
            executable='visual_odom',
            name='visual_odom',
            namespace=ns,
            output='screen',
            parameters=[{'calibration_file':param_file_path}],
        ),
                
    ])