import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = os.environ.get('ROBOT_NAMESPACE', os.environ.get('ROBOT', '')).strip()

    return LaunchDescription([
        Node(
            package='comms_modem_monitor',
            executable='modem_monitor.py',
            name='modem_monitor',
            namespace=robot_namespace,
            parameters=[
                {'rate_hz': 2.0},
                {'topic_name': 'modem_link'},
            ],
        )
    ])
