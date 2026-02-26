from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'
        ),
        
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            #namespace=namespace,
            name="laserscan_filter",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("rona_navigation"),
                    "config", "angular_filter.yaml",
                ])],
            
        )
    ])
