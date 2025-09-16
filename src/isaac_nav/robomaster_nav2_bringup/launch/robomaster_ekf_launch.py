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
import socket

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_share = get_package_share_directory('robomaster_nav2_bringup')

    prefix = socket.gethostname()

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=prefix,
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    imu_cmd = Node(
        package='robomaster_bno085',
        executable='bno085_node',
        name='bno085_node',
        output='screen',
        namespace=prefix,
        parameters=[{"use_sim_time" : use_sim_time}],
    )

    ekf_config = os.path.join(
        bringup_share,
        'config',
        'robot_localization_ekf.yaml'
    )

    ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace=prefix,
        parameters=[ekf_config, {
            "use_sim_time" : use_sim_time,
            "odom_frame": prefix + "/odom",
            "base_link_frame": prefix + "/base_footprint",
            "world_frame": prefix + "/odom"            
            }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(imu_cmd)
    ld.add_action(ekf_cmd)

    return ld
