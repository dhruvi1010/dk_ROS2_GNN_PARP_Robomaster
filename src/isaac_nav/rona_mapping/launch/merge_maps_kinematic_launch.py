from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rona_mapping',
            executable='merge_maps_kinematic',
            name='rona_mapping',
            output='screen'
        )
    ])
