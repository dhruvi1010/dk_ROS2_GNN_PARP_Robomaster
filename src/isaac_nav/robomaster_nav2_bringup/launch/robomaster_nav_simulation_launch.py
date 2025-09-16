# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Get the launch directory
    description_share = get_package_share_directory('ros_robomaster_description')
    gazebo_share = get_package_share_directory('robomaster_gazebo')
    bringup_share = get_package_share_directory('robomaster_nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_share, 'config', 'robomaster_nav2_params.yaml'),
        # default_value=os.path.join(bringup_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(description_share, 'urdf', 'robomaster.xacro'),
        description='Full path to the urdf file to use for launched nodes')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_share, 'config', 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # default_value=os.path.join(gazebo_share, 'worlds', 'pacelab.world'),
        default_value=os.path.join(gazebo_share, 'worlds', 'logimat.world'),
        # default_value=os.path.join(gazebo_share, 'worlds', 'pacelab_amazon_pods.world'),
        description='Full path to world model file to load')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        description='Whether run a SLAM (uses AMCL instead if false)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'logimat_gazebo', 'logimat.yaml'),
        description='Full path to map yaml file to load')

    # Specify the actions
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[bringup_share], output='screen')

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'lauch', 'ros_robomaster_description.launch.py')),
        launch_arguments={'namespace:': '', 
                          'use_sim_time': use_sim_time,
                          'urdf_file': '',
                          'prefix': '',
                          'urdf_file': urdf_file,
                          'use_joint_tester': 'true'                          
                          }
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
            }])

    spawn_entity_cmd = Node(
    	package='ros_gz_sim', 
    	executable='create',
        arguments=['-name', 'robomaster1', '-topic', '/robot_description', '-x', '1.0', '-y', '1.0'],
        output='screen'
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch','rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    namespaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ''})

    bringup_cmd_group = GroupAction([
        # Node(
        #     parameters=[
        #         namespaced_params_file,
        #         {"scan_topic" : "/scan"},
        #         {"odom_frame" : "odom"},
        #         {"map_frame" : "map"},
        #         {"base_frame" : "base_footprint"},
        #         {"use_sim_time" : use_sim_time},
        #     ],
        #     package='slam_toolbox',
        #     executable='lifelong_slam_toolbox_node',
        #     name='Slam_Toolbox_Mapping',
        #     output='screen'
        # ),        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')),
            condition=IfCondition(PythonExpression([use_slam])),
            launch_arguments={'namespace': '',
                              'scan_topic' : '/scan',
                              'odom_frame' : 'odom',
                              'map_frame' : 'map',
                              'base_frame' : 'base_footprint',
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': namespaced_params_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'amcl_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_slam])),
            launch_arguments={'namespace': '',
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': namespaced_params_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'navigation_launch.py')),
            launch_arguments={'namespace': '',
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': namespaced_params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items())
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_urdf_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Add the actions to launch simulator and all of the navigation nodes
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
