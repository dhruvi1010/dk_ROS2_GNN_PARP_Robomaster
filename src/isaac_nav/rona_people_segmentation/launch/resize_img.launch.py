from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_proc',
            executable='resize_node',
            name='resize_node',
            remappings=[
                ('image/image_raw', '/ep02/camera0/color/image_raw'),
                ('image/camera_info', '/ep02/camera0/color/camera_info'),
                ('resized/image_raw', '/output/resized_image'),
                ('resized/camera_info', '/output/resized_camera_info'),
            ],
            parameters=[{
                'use_scale': False,
                'width': 640,
                'height': 480,
                'interpolation': 1  # Linear interpolation
            }]
        )
    ])
