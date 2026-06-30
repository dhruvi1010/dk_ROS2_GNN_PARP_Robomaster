import os
import socket
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
    ExecuteProcess, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString


# ---------------------------------------------------------------------------
# Rosbag recorder built via OpaqueFunction.
#
# Why an OpaqueFunction instead of PathJoinSubstitution + [prefix, '/plan']?
# The Humble launch package in this container rejects nested lists inside
# PathJoinSubstitution and ExecuteProcess.cmd with:
#   "Failed to normalize given item of type '<class 'list'>'".
# Resolving LaunchConfigurations to plain strings at launch time (inside
# the OpaqueFunction) sidesteps the normalization issue entirely.
# ---------------------------------------------------------------------------
def _make_rosbag_action(context, *args, **kwargs):
    record = LaunchConfiguration('record_rosbag').perform(context).lower()
    if record not in ('true', '1', 'yes'):
        return []  # bag recording disabled

    bag_name_val = LaunchConfiguration('test_bag_name').perform(context)
    bags_dir_val = LaunchConfiguration('bags_dir').perform(context)
    prefix_val   = LaunchConfiguration('prefix').perform(context)

    # Sanitize hostname so it works both as a ROS namespace and a filesystem dir.
    host_val = socket.gethostname().replace('-', '_')
    ts_val   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    bag_path = os.path.join(bags_dir_val, host_val,
                            f'{bag_name_val}_{ts_val}_bag')

    p = prefix_val  # short alias for readability below
    cmd = [
        'ros2', 'bag', 'record',
        '-o', bag_path,
        '-s', 'mcap',                       # MCAP, same as bag_record_all.launch.py
        # 1. plans + path metrics
        f'/{p}/plan',
        f'/{p}/local_plan',
        f'/{p}/transformed_global_plan',
        f'/{p}/received_global_plan',
        # 2. costmaps (raw + compressed + updates)
        f'/{p}/global_costmap/costmap',
        f'/{p}/global_costmap/costmap_raw',
        f'/{p}/global_costmap/costmap_updates',
        f'/{p}/local_costmap/costmap',
        f'/{p}/local_costmap/costmap_raw',
        # 3. perception — RELAYED single-type (NEVER raw /tracked_polygons)
        '/tracked_polygons_logged',
        # 4. comms (L1 evidence) + battery (energy)
        f'/{p}/comms/link_stats',
        f'/{p}/fake_rsrp',
        f'/{p}/battery_state',
        # 5. route_cost outputs (the dissertation core)
        f'/{p}/route_cost',
        f'/{p}/route_puc',
        f'/{p}/route_puc_components',
        # 6. pose / ground truth
        f'/{p}/odom',
        f'/{p}/odom_vicon',
        f'/{p}/vicon_pose',
        f'/{p}/cmd_vel',
        f'/{p}/cmd_wheel_speed',
        f'/{p}/scan_filtered',
        # 7. tf + navigation action
        '/tf', '/tf_static',
        f'/{p}/behavior_tree_log',
        '/navigate_to_pose/feedback',
        '/navigate_to_pose/result',
        '/diagnostics',
    ]

    print(f'[rosbag_setup] host={host_val}  ns=/{p}  bag={bag_name_val}  '
          f'topics={len(cmd) - 7}  ->  {bag_path}')
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():

    bringup_share = get_package_share_directory('rona_navigation')

    # =========================
    # Launch Arguments
    # =========================

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        # env ROBOT wins (lets you fake an identity for sim trials);
        # otherwise fall back to the actual hostname (sanitized for ROS).
        default_value=os.environ.get("ROBOT",
                                     socket.gethostname().replace('-', '_')),
        description='Robot namespace (env ROBOT > hostname).'
    )

    prefix = LaunchConfiguration('prefix')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # nav2 stack: reuse L2's yaml (unchanged) — Phase 3 adds no nav2 plugin
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_share, 'config',
            'o11y_parp_gnn_robomaster_nav2_radar.yaml'
        ),
        description='Nav2 parameters file (L2 o11y_risk_layer enabled)'
    )

    # route_cost_puc params (dedicated, NOT a nav2 copy)
    declare_rcp_params_cmd = DeclareLaunchArgument(
        'route_cost_params_file',
        default_value=os.path.join(bringup_share, 'config',
                                   'route_cost_puc_params.yaml'),
        description='route_cost_puc_pynode params YAML')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            bringup_share, 'config', 'navigate_w_recovery.xml'
        ),
        description='Behavior tree XML file'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_share, 'maps', 'map.yaml'),
        description='Map YAML file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_share, 'config', 'rviz', 'nav2_robomaster_view.rviz'
        ),
        description='RVIZ config file'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ'
    )

    declare_test_bag_name_cmd = DeclareLaunchArgument(
        'test_bag_name', default_value='default_run',
        description='Per-trial bag/CSV label; also fed to RouteCost.trial_id. '
                    'Independent of the 5G inference run_id.')

    declare_record_rosbag_cmd = DeclareLaunchArgument(
        'record_rosbag', default_value='false',
        description='If true, ros2 bag record runs alongside the trial.')

    # Storage directory for rosbag + CSV (container path; bind-mounted to host
    # /home/robot_3/isaac_ros-dev/dk_ros2_bags/). Override with bags_dir:=...
    declare_bags_dir_cmd = DeclareLaunchArgument(
        'bags_dir',
        default_value='/workspaces/isaac_ros-dev/dk_ros2_bags',
        description='Output dir for the per-trial rosbag and CSV.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rcp_params_file = LaunchConfiguration('route_cost_params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    test_bag_name = LaunchConfiguration('test_bag_name')
    record_rosbag = LaunchConfiguration('record_rosbag')
    bags_dir = LaunchConfiguration('bags_dir')

    # =========================
    # Namespaced Params
    # =========================

    namespaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': [prefix, '/']}
    )

    # =========================
    # RVIZ (optional)
    # =========================

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rviz', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': prefix,
            'use_namespace': 'True',
            'rviz_config': rviz_config_file
        }.items()
    )

    # =========================
    # Comms Monitor Node
    # =========================

    comms_monitor_node = Node(
        package='comms_monitor_pynode',
        executable='comms_monitor_pynode',
        name='comms_monitor',
        output='screen',
        parameters=[
            {'edge_ip': '172.16.3.62'},
            {'edge_port': 5005},
            {'rsrp_topic': 'fake_rsrp'},
            #{'robot_id': 'rm03'},
            #{'interface': 'wwan0'},      # 5G interface name (wwan0) if needed
        ]
    )

    # =========================
    # Route Cost / PUC Node (Phase 3) + CSV logger
    # =========================

    route_cost_puc_node = Node(
        package='route_cost_puc_pynode',
        executable='route_cost_puc_node',
        name='route_cost_puc',
        output='screen',
        parameters=[rcp_params_file,
                    {'trial_id': test_bag_name}])

    csv_logger_node = Node(
        package='route_cost_puc_pynode',
        executable='route_cost_csv_logger',
        name='route_cost_csv_logger',
        output='screen',
        parameters=[{'run_id': test_bag_name},
                    {'output_dir': bags_dir}])

    # =========================
    # Main Group (Single Namespace)
    # =========================

    bringup_cmd_group = GroupAction([

        PushRosNamespace(prefix),

        # Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'rviz',
                             'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': prefix,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': namespaced_params_file,
                'default_bt_xml_filename': default_bt_xml_filename,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true'
            }.items()
        ),

        # Laser Scan Filter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch',
                             'laserscan_filter.launch.py')
            ),
            launch_arguments={'namespace': prefix}.items()
        ),

        comms_monitor_node,   # Communication Monitor
        route_cost_puc_node,  # Route Cost / PUC Node
        csv_logger_node,      # CSV Logger for Route Cost / PUC data
    ])

    # ---------- optional rosbag relay + recorder ----------
    #
    # Relay dual-type /tracked_polygons -> single-type /tracked_polygons_logged
    # so rosbag2 can record it. (Source is dual-type because foreign rviz subs
    # advertise PolygonStamped — intentional, not cleanable.) Runs only when
    # recording is enabled.
    relay_cmd = ExecuteProcess(
        condition=IfCondition(record_rosbag),
        cmd=['python3',
             '/workspaces/isaac_ros-dev/dk_ros2_bags/tracked_polygons_relay_node.py'],
        output='screen')

    # The rosbag recorder is built lazily by an OpaqueFunction so all
    # LaunchConfigurations resolve to plain strings before the cmd list is
    # constructed. See _make_rosbag_action() at the top of this file.
    rosbag_cmd = OpaqueFunction(function=_make_rosbag_action)

    # =========================
    # Final Launch Description
    # =========================

    ld = LaunchDescription()
    for c in (declare_prefix_cmd, declare_use_sim_time_cmd, declare_params_file_cmd,
              declare_rcp_params_cmd, declare_bt_xml_cmd, declare_map_yaml_cmd,
              declare_autostart_cmd, declare_rviz_config_file_cmd, declare_use_rviz_cmd,
              declare_test_bag_name_cmd, declare_record_rosbag_cmd, declare_bags_dir_cmd):
        ld.add_action(c)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(relay_cmd)      # start relay BEFORE recorder so the topic is discoverable
    ld.add_action(rosbag_cmd)
    return ld
