import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString


def generate_launch_description():

    bringup_share = get_package_share_directory('rona_navigation')

    # =========================
    # Launch Arguments
    # =========================

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value=os.environ.get("ROBOT", "rm03"),
        description='Robot namespace'
    )

    prefix = LaunchConfiguration('prefix')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_share,
            'config',
            'o11y_parp_gnn_robomaster_nav2_radar.yaml'
        ),
        description='Nav2 parameters file (L2 o11y_risk_layer enabled)'
    )

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

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

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

        # Communication Monitor
        comms_monitor_node,
    ])

    # =========================
    # Final Launch Description
    # =========================

    ld = LaunchDescription()

    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)

    return ld