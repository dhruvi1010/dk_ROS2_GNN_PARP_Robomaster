import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    package_share = get_package_share_directory('ros_robomaster_description')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_joint_tester = LaunchConfiguration('use_joint_tester')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value= os.path.join(package_share, 'urdf', 'robomaster.xacro'),
        description='Full path to the urdf file to use for launched nodes')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value=[namespace, '/'],
        description='TF prefix')
    declare_use_joint_tester_cmd = DeclareLaunchArgument(
        'use_joint_tester',
        default_value='false',
        description='Whether to launch joint_tester')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_urdf_file_cmd,
        declare_namespace_cmd,
        declare_prefix_cmd,
        declare_use_joint_tester_cmd,
        Node( 
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                'robot_description': Command(['xacro ', urdf_file]), 
                'frame_prefix' : [namespace, '/']
            }],
            remappings=[('joint_states', 'wheel_joint_states')],
            arguments=[urdf_file]),
        Node( 
            condition=IfCondition(use_joint_tester),
            package='ros_robomaster_description',
            executable='joint_tester',
            namespace=namespace,
            name='joint_tester',
            output='screen'),
    ])
