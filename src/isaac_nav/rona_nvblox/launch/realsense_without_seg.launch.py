from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu


from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

from launch.substitutions import TextSubstitution

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '',
                 description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=[
                 'debug', 'info', 'warn'], cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg('camera_serial_numbers', '',
                 description='List of the serial no of the extra cameras. (comma separated)',
                 cli=True)
    args.add_arg(
        'multicam_urdf_path',
        lu.get_path('rona_nvblox',
                    'config/urdf/4_realsense_carter_example_calibration.urdf.xacro'),
        description='Path to a URDF file describing the camera rig extrinsics. Only used in multicam.',
        cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    # args.add_arg(
    #     'people_segmentation',
    #     default=NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
    #     choices=[
    #         str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
    #         str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)
    #     ],
    #     description='The  model type of PeopleSemSegNet (only used when mode:=people_segmentation).',
    #     cli=True)
    args.add_arg(
        'attach_to_container',
        'False',
        description='Add components to an existing component container.',
        cli=True)
    args.add_arg(
        'run_realsense',
        'True',
        description='Launch Realsense drivers')
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
    args.add_arg(
        'namespace',
        'robot0',
        description='Namespace to push all nodes under.',
        cli=True)
    
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))

    # Single or Multi-realsense
    is_multi_cam = UnlessCondition(lu.is_equal(args.num_cameras, '1'))
    camera_mode = lu.if_else_substitution(
        lu.is_equal(args.num_cameras, '1'),
        str(NvbloxCamera.realsense),
        str(NvbloxCamera.multi_realsense)
    )
    # Only up to 4 Realsenses is supported.
    actions.append(
        lu.assert_condition(
            'Up to 4 cameras have been tested! num_cameras must be less than 5.',
            IfCondition(PythonExpression(['int("', args.num_cameras, '") > 4']))),
    )

    run_rs_driver = UnlessCondition(
        OrSubstitution(lu.is_valid(args.rosbag), lu.is_false(args.run_realsense)))
    # Realsense
    actions.append(
        lu.include(
            'rona_nvblox',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'container_name': args.namespace,
                'camera_serial_numbers': args.camera_serial_numbers,
                'num_cameras': args.num_cameras,
                'namespace': args.namespace,
            },
            condition=run_rs_driver))

    # Visual SLAM
    actions.append(
        lu.include(
            'rona_nvblox',
            'launch/perception/vslam.launch.py',
            launch_arguments={
                'container_name': args.namespace,
                'camera': camera_mode,
                'namespace': args.namespace,
            },
            # Delay for 1 second to make sure that the static topics from the rosbag are published.
            delay=1.0,
        ))
    # People detection for multi-RS
    camera_namespaces = ['camera0', 'camera1', 'camera2', 'camera3']
    camera_input_topics = []
    input_camera_info_topics = []
    output_resized_image_topics = []
    output_resized_camera_info_topics = []
    for ns in camera_namespaces:
        camera_input_topics.append(f'/{ns}/color/image_raw')
        input_camera_info_topics.append(f'/{ns}/color/camera_info')
        output_resized_image_topics.append(f'/{ns}/segmentation/image_resized')
        output_resized_camera_info_topics.append(f'/{ns}/segmentation/camera_info_resized')

    # People segmentation
    # actions.append(
    #     lu.include(
    #         'rona_nvblox',
    #         'launch/perception/segmentation.launch.py',
    #         launch_arguments={
    #             'container_name': args.container_name,
    #             'people_segmentation': args.people_segmentation,
    #             'namespace_list': camera_namespaces,
    #             'input_topic_list': camera_input_topics,
    #             'input_camera_info_topic_list': input_camera_info_topics,
    #             'output_resized_image_topic_list': output_resized_image_topics,
    #             'output_resized_camera_info_topic_list': output_resized_camera_info_topics,
    #             'num_cameras': args.num_cameras,
    #             # fixing rosbag replay dropping fps
    #             'one_container_per_camera': True
    #         },
    #         condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_segmentation))))

    # People detection
    actions.append(
        lu.include(
            'rona_nvblox',
            'launch/perception/detection.launch.py',
            launch_arguments={
                'namespace_list': camera_namespaces,
                'input_topic_list': camera_input_topics,
                'num_cameras': args.num_cameras,
                'container_name': args.namespace,
                # fixing rosbag replay dropping fps
                'one_container_per_camera': True
            },
            condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_detection))))

    # Nvblox
    actions.append(
        lu.include(
            'rona_nvblox',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': args.namespace,
                'mode': args.mode,
                'camera': camera_mode,
                'num_cameras': args.num_cameras,
                'namespace': args.namespace,
            }))

    # TF transforms for multi-realsense
    actions.append(
        lu.add_robot_description(robot_calibration_path=args.multicam_urdf_path,
                                 condition=is_multi_cam)
    )

    # Play ros2bag
    actions.append(
        lu.play_rosbag(
            bag_path=args.rosbag,
            additional_bag_play_args=args.rosbag_args,
            condition=IfCondition(lu.is_valid(args.rosbag))))

    # Visualization
    # actions.append(
    #     lu.include(
    #         'rona_nvblox',
    #         'launch/visualization/visualization.launch.py',
    #         launch_arguments={
    #             'mode': args.mode,
    #             'camera': camera_mode,
    #             'use_foxglove_whitelist': args.use_foxglove_whitelist,
    #         }))

    # Component container
    actions.append(
        lu.component_container(
            args.namespace, condition=UnlessCondition(args.attach_to_container),
            log_level=args.log_level))

    # static tf for base_footprint and camera0_link
    
    actions.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera0_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', [args.namespace, '/base_footprint'], [args.namespace, '/camera0_link']],
            namespace=args.namespace,
            output='screen',
        )
    )

    return LaunchDescription(actions)
