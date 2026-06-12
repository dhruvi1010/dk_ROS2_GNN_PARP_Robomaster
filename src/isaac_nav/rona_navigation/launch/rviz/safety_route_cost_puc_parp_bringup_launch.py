import os
import socket
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString


def generate_launch_description():

    bringup_share = get_package_share_directory('rona_navigation')
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # =========================
    # Launch Arguments
    # =========================

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value=os.environ.get("ROBOT", "default"),
        description='Robot namespace'
    )

    prefix = LaunchConfiguration('prefix')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # nav2 stack: reuse L2's yaml (unchanged) — Phase 3 adds no nav2 plugin
    # but does add a separate route_cost_puc_pynode with its own params YAML for the GNN and PUC components = route_cost_puc node + CSV + rosbag
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_share,
            'config',
            'safety_parp_gnn_robomaster_nav2_radar.yaml'
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
            bringup_share,
            'config',
            'navigate_w_recovery.xml'
        ),
        description='Behavior tree XML file'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_share,
            'maps',
            'map.yaml'
        ),
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
            bringup_share,
            'config',
            'rviz',
            'nav2_robomaster_view.rviz'
        ),
        description='RVIZ config file'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ'
    )


    declare_run_id_cmd = DeclareLaunchArgument(
        'run_id', default_value='default_run',
        description='Per-trial id propagated into RouteCost.trial_id and the rosbag dir.')
    
    declare_record_rosbag_cmd = DeclareLaunchArgument(
        'record_rosbag', default_value='false',
        description='If true, ros2 bag record runs alongside the trial.')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rcp_params_file = LaunchConfiguration('route_cost_params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    run_id = LaunchConfiguration('run_id')
    record_rosbag = LaunchConfiguration('record_rosbag')
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
            os.path.join(
                bringup_share,
                'launch',
                'rviz',
                'rviz_launch.py'
            )
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
    # Route Cost / PUC Node (Phase 3) + ROSBAG  CSV logger
    # =========================

    route_cost_puc_node = Node(
        package='route_cost_puc_pynode',
        executable='route_cost_puc_node',
        name='route_cost_puc',
        output='screen',
        parameters=[rcp_params_file,
                    {'trial_id': run_id}, {'robot_id': prefix}])
        # parameters=[
        #     os.path.join(bringup_share, 'config', 'route_cost_puc_params.yaml'),
        #     {'trial_id': LaunchConfiguration('run_id')},   # per-trial override of the yaml default
        # ]
    #)   

     # Storage directory for rosbag + CSV (container path; bind-mounted to host
    # /home/robot_3/isaac_ros-dev/dk_ros2_bags/). Override with bags_dir:=...
    declare_bags_dir_cmd = DeclareLaunchArgument(
        'bags_dir',
        default_value='/workspaces/isaac_ros-dev/dk_ros2_bags',
        description='Output dir for the per-trial rosbag and CSV.')
    bags_dir = LaunchConfiguration('bags_dir')

    

    # ---------- bringup group (single namespace) ----------

    csv_logger_node = Node(
        package='route_cost_puc_pynode',
        executable='route_cost_csv_logger',
        name='route_cost_csv_logger',
        output='screen',
        parameters=[{'run_id': run_id},
                    {'output_dir': bags_dir}, {'robot_id': prefix}])

    # =========================
    # Main Group (Single Namespace)
    # =========================

    bringup_cmd_group = GroupAction([

        PushRosNamespace(prefix),

        # Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share,
                    'launch',
                    'rviz',
                    'navigation_launch.py'
                )
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
                os.path.join(
                    bringup_share,
                    'launch',
                    'laserscan_filter.launch.py'
                )
            ),
            launch_arguments={
                'namespace': prefix
            }.items()
        ),

        comms_monitor_node,  # Communication Monitor
        route_cost_puc_node,  # Route Cost / PUC Node
        csv_logger_node,     # CSV Logger for Route Cost / PUC data
    ])

        # ---------- optional rosbag record (mirrors edge multi_robot_inference style) ----------

    # Relay dual-type /tracked_polygons -> single-type /tracked_polygons_logged so
    # rosbag2 can record it (source is dual-type because foreign rviz subs advertise
    # PolygonStamped — intentional, not cleanable). Only runs when recording.
    relay_cmd = ExecuteProcess(
        condition=IfCondition(record_rosbag),
        cmd=['python3',
             '/workspaces/isaac_ros-dev/dk_ros2_bags/tracked_polygons_relay_node.py'],
        output='screen')

    bag_dir = PathJoinSubstitution([
        bags_dir, TextSubstitution(text='/'),
        run_id, TextSubstitution(text=f'_{ts}_bag')])

    rosbag_cmd = ExecuteProcess(
        condition=IfCondition(record_rosbag),
        cmd=['ros2', 'bag', 'record', '-o', bag_dir,
             '/rm03/plan', '/rm03/local_plan',
             '/rm03/global_costmap/costmap', '/rm03/global_costmap/costmap_raw',
             '/rm03/local_costmap/costmap',
             '/tracked_polygons_logged',
             '/rm03/comms/link_stats', '/rm03/battery_state',
             '/rm03/route_cost', '/rm03/route_puc', '/rm03/route_puc_components',
             '/rm03/odom', '/rm03/vicon_pose',
             '/tf', '/tf_static',
             '/navigate_to_pose/feedback', '/navigate_to_pose/result'],
        output='screen')



    # =========================
    # Final Launch Description
    # =========================

    # ld = LaunchDescription()

    # ld.add_action(declare_prefix_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_bt_xml_cmd)
    # ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(declare_autostart_cmd)
    # ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(declare_run_id_cmd)
    # ld.add_action(declare_record_rosbag_cmd)
    # ld.add.act

    # ld.add_action(rviz_cmd)
    # ld.add_action(bringup_cmd_group)
    # ld.add_action(rosbag_cmd)
    # return ld


    ld = LaunchDescription()
    for c in (declare_prefix_cmd, declare_use_sim_time_cmd, declare_params_file_cmd,
              declare_rcp_params_cmd, declare_bt_xml_cmd, declare_map_yaml_cmd,
              declare_autostart_cmd, declare_rviz_config_file_cmd, declare_use_rviz_cmd,
              declare_run_id_cmd, declare_record_rosbag_cmd, declare_bags_dir_cmd):
        ld.add_action(c)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(relay_cmd)
    ld.add_action(rosbag_cmd)
    return ld
