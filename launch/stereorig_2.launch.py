import os
import yaml
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def bag_exists(time_cap):
    file_path = '/ws/data/'
    num = 0
    for dirpath, dirnames, filenames in os.walk(file_path):
        for dirname in dirnames:
            if time_cap in dirname:
                num += 1
    return num

def generate_launch_description():
    ct = datetime.datetime.now()
    ct_str = ct.strftime("%Y-%m-%d-%H_%M_%S")
    num_files = bag_exists(ct_str)
    if num_files > 0:
        ct_str = f"{ct_str}_{num_files}"

    launch_params_path = os.path.join('/ws/data/config/parms.yaml')
    with open(launch_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)

    # Extracting parameters
    jetson_params = launch_params["jetson_2"]['ros_parameters']
    camera_type = jetson_params['camera_type']
    serial = jetson_params['serial']
    sonar = jetson_params['sonar']
    cam_topic = jetson_params['cam_topic']
    debug = jetson_params['debug']
    compute_brightness = jetson_params['compute_brightness']
    adjust_timestamp = jetson_params['adjust_timestamp']
    dump_node_map = jetson_params['dump_node_map']
    gain_auto = jetson_params['gain_auto']
    exposure_auto = jetson_params['exposure_auto']
    user_set_selector = jetson_params['user_set_selector']
    user_set_load = jetson_params['user_set_load']
    frame_rate_auto = jetson_params['frame_rate_auto']
    frame_rate = jetson_params['frame_rate']
    frame_rate_enable = jetson_params['frame_rate_enable']
    buffer_queue_size = jetson_params['buffer_queue_size']
    trigger_mode = jetson_params['trigger_mode']
    chunk_mode_active = jetson_params['chunk_mode_active']
    chunk_selector_frame_id = jetson_params['chunk_selector_frame_id']
    chunk_enable_frame_id = jetson_params['chunk_enable_frame_id']
    chunk_selector_exposure_time = jetson_params['chunk_selector_exposure_time']
    chunk_enable_exposure_time = jetson_params['chunk_enable_exposure_time']
    chunk_selector_gain = jetson_params['chunk_selector_gain']
    chunk_enable_gain = jetson_params['chunk_enable_gain']
    chunk_selector_timestamp = jetson_params['chunk_selector_timestamp']
    chunk_enable_timestamp = jetson_params['chunk_enable_timestamp']
    adc_bit_depth = jetson_params['adc_bit_depth']
    namespace = LaunchConfiguration('namespace')

    # Paths to various files
    microstrain_launch_file = os.path.join(
        get_package_share_directory('microstrain_inertial_examples'),
        'launch', 'cv7_launch.py'
    )
    cv7_params_file = os.path.join(
        get_package_share_directory('microstrain_inertial_examples'),
        'config', 'cv7', 'cv7.yml'
    )
    sonar_config = os.path.join(
        get_package_share_directory('imagenex831l_ros2'),
        'cfg', 'sonar.yaml'
    )
    cam_dir = get_package_share_directory('spinnaker_camera_driver')
    sonar_dir = get_package_share_directory('imagenex831l_ros2')
    screen_dir = get_package_share_directory('custom_guyi')
    tag_dir = get_package_share_directory('ros2_aruco')

    # Group actions and nodes
    included_cam_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(cam_dir, 'launch', 'driver_node.launch.py')),
                launch_arguments={
                    'camera_type': camera_type,
                    'serial': serial,
                    'debug': debug,
                    'compute_brightness': compute_brightness,
                    'adjust_timestamp': adjust_timestamp,
                    'dump_node_map': dump_node_map,
                    'gain_auto': gain_auto,
                    'exposure_auto': exposure_auto,
                    'user_set_selector': user_set_selector,
                    'user_set_load': user_set_load,
                    'frame_rate_auto': frame_rate_auto,
                    'frame_rate': frame_rate,
                    'frame_rate_enable': frame_rate_enable,
                    'buffer_queue_size': buffer_queue_size,
                    'trigger_mode': trigger_mode,
                    'chunk_mode_active': chunk_mode_active,
                    'chunk_selector_frame_id': chunk_selector_frame_id,
                    'chunk_enable_frame_id': chunk_enable_frame_id,
                    'chunk_selector_exposure_time': chunk_selector_exposure_time,
                    'chunk_enable_exposure_time': chunk_enable_exposure_time,
                    'chunk_selector_gain': chunk_selector_gain,
                    'chunk_enable_gain': chunk_enable_gain,
                    'chunk_selector_timestamp': chunk_selector_timestamp,
                    'chunk_enable_timestamp': chunk_enable_timestamp
                }.items()
            )
        ]
    )

    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        namespace=namespace,
        output="screen"
    )

    base_to_range = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        namespace=namespace,
        arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    included_imu_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(microstrain_launch_file),
                launch_arguments={'namespace': namespace}.items()
            )
        ]
    )

    included_sonar_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sonar_dir, 'launch', 'sonar.launch.py')),
                launch_arguments={'sonar': sonar, 'device': device, 'config': sonar_config}.items()
            )
        ]
    )
    
    included_screen_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(screen_dir, 'launch', 'gui.launch.py')),
                launch_arguments={'cam_topic': cam_topic, 'device': device}.items(),
            )
        ]
    )

    included_tag_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(tag_dir, 'launch', 'aruco_recognition.launch.py')),
                launch_arguments={'image_topic': cam_topic, 'aruco_dictionary_id': 'DICT_ARUCO_ORIGINAL', 'camera_info_topic': '/image_raw/camera_info'}.items(),
            )
        ]
    )

    rosbag_node = Node(
        package='ros2_bringup',
        executable='rosbag.py',
        namespace=namespace,
        arguments=[device]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        included_tag_launch,
        included_cam_launch,
        rosbag_node,
        ping1d_node,
        base_to_range,
        included_imu_launch,
        included_sonar_launch,
        included_screen_launch
    ])
