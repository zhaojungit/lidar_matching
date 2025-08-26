from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('lidar_matching')
    
    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    
    # Declare the launch argument
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'params', 'lidar_2d.yaml'),
        description='Path to the configuration file'
    )
    
    # Create the node
    lidar_matching_node = Node(
        package='lidar_matching',
        executable='2d_mapping',
        name='lidar_matching',
        output='screen',
        parameters=[config_file]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_config_file_cmd,
        lidar_matching_node
    ])
