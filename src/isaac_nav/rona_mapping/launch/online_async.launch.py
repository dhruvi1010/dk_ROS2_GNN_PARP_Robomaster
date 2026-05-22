import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    prefix = LaunchConfiguration('prefix')

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value= os.environ.get("ROBOT", "default"),
        description='Namespace prefix')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', # Using real time by default
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rona_mapping"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    namespaced_slam_params_file = ReplaceString(
        source_file=slam_params_file,
        replacements={'<robot_namespace>': [prefix, '/']})

    start_async_slam_toolbox_node = Node(
        parameters=[
          namespaced_slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=prefix,
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
