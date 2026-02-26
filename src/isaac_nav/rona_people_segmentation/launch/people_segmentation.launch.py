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
            package="rona_people_segmentation",
            executable="people_segmentation_node",
            namespace=namespace,
            name="rona_people_segmentation",
            remappings=[
                ('image_raw', 'camera0/color/image_raw'),
            ],
        )
    ])
