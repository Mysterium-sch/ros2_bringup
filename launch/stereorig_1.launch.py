import os
import launch
import yaml
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def bag_exists(time_cap):
    file_path = '/ws/data/'
    num= 0
    for dirpath, dirnames, filenames in os.walk(file_path):
        for dirname in dirnames:
            if time_cap in dirname:
                num = num + 1
    return num

def generate_launch_description():

    ct = datetime.datetime.now()
    ct_str = ct.strftime("%Y-%m-%d-%H_%M_%S")
    num_files = bag_exists(ct_str)
    if num_files > 0:
        ct_str = ct_str + "_" + str(num)

    launch_params_path = os.path.join('/ws/data/config/parms.yaml')
    with open(launch_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)

    camera_type = launch_params["jetson_1"]['ros_parameters']['camera_type']
    device = launch_params["jetson_1"]['ros_parameters']['device']
    serial = launch_params["jetson_1"]['ros_parameters']['serial']
    sonar = launch_params["jetson_1"]['ros_parameters']['sonar']
    cam_topic = launch_params["jetson_1"]['ros_parameters']['cam_topic']
    debug = launch_params["jetson_1"]['ros_parameters']['debug']
    compute_brightness = launch_params["jetson_1"]['ros_parameters']['compute_brightness']
    adjust_timestamp = launch_params["jetson_1"]['ros_parameters']['adjust_timestamp']
    dump_node_map = launch_params["jetson_1"]['ros_parameters']['dump_node_map']
    pixel_format = launch_params["jetson_1"]['ros_parameters']['pixel_format']
    gain_auto = launch_params["jetson_1"]['ros_parameters']['gain_auto']
    exposure_auto = launch_params["jetson_1"]['ros_parameters']['exposure_auto']
    user_set_selector = launch_params["jetson_1"]['ros_parameters']['user_set_selector']
    user_set_load = launch_params["jetson_1"]['ros_parameters']['user_set_load']
    frame_rate_auto = launch_params["jetson_1"]['ros_parameters']['frame_rate_auto']
    frame_rate = launch_params["jetson_1"]['ros_parameters']['frame_rate']
    frame_rate_enable = launch_params["jetson_1"]['ros_parameters']['frame_rate_enable']
    buffer_queue_size = launch_params["jetson_1"]['ros_parameters']['buffer_queue_size']
    trigger_mode = launch_params["jetson_1"]['ros_parameters']['trigger_mode']
    chunk_mode_active = launch_params["jetson_1"]['ros_parameters']['chunk_mode_active']
    chunk_selector_frame_id = launch_params["jetson_1"]['ros_parameters']['chunk_selector_frame_id']
    chunk_enable_frame_id = launch_params["jetson_1"]['ros_parameters']['chunk_enable_frame_id']
    chunk_selector_exposure_time = launch_params["jetson_1"]['ros_parameters']['chunk_selector_exposure_time']
    chunk_enable_exposure_time = launch_params["jetson_1"]['ros_parameters']['chunk_enable_exposure_time']
    chunk_selector_gain = launch_params["jetson_1"]['ros_parameters']['chunk_selector_gain']
    chunk_enable_gain = launch_params["jetson_1"]['ros_parameters']['chunk_enable_gain']
    chunk_selector_timestamp = launch_params["jetson_1"]['ros_parameters']['chunk_selector_timestamp']
    chunk_enable_timestamp = launch_params["jetson_1"]['ros_parameters']['chunk_enable_timestamp']
    adc_bit_depth = launch_params["jetson_1"]['ros_parameters']['adc_bit_depth']
    namespace = LaunchConfiguration('namespace')

    _MICROSTRAIN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_examples'),
        'launch', 'cv7_launch.py'
    )
    _CV7_PARAMS_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_examples'),
        'config', 'cv7', 'cv7.yml'
    )
    _EMPTY_PARAMS_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_driver'),
        'config', 'empty.yml'
    )
    
    config = os.path.join(get_package_share_directory('imagenex831l_ros2'),
        'cfg',
        'sonar.yaml'
    )

    cam_dir = get_package_share_directory('spinnaker_camera_driver')
    included_cam_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(cam_dir, 'launch', 'driver_node.launch.py')),
                launch_arguments={'camera_type': camera_type, 'serial': serial,'debug': debug,'compute_brightness': compute_brightness,'adjust_timestamp': adjust_timestamp,'dump_node_map': dump_node_map,'gain_auto': gain_auto,'exposure_auto': exposure_auto,'user_set_selector': user_set_selector,'user_set_load': user_set_load,'frame_rate_auto': frame_rate_auto,'frame_rate': frame_rate,'frame_rate_enable': frame_rate_enable,'buffer_queue_size': buffer_queue_size,'trigger_mode': trigger_mode,'chunk_mode_active': chunk_mode_active,'chunk_selector_frame_id': chunk_selector_frame_id,'chunk_enable_frame_id': chunk_enable_frame_id,'chunk_selector_exposure_time': chunk_selector_exposure_time,'chunk_enable_exposure_time': chunk_enable_exposure_time,'chunk_selector_gain': chunk_selector_gain,'chunk_enable_gain': chunk_enable_gain,'chunk_selector_timestamp': chunk_selector_timestamp,'chunk_enable_timestamp': chunk_enable_timestamp}.items()
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
                PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
                launch_arguments={'namespace': namespace}.items()
            )
        ]
    )

    sonar_dir = get_package_share_directory('imagenex831l_ros2')
    included_sonar_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sonar_dir, 'launch', 'sonar.launch.py')),
                launch_arguments={'sonar': sonar, 'device': device, 'config': config}.items()
            )
        ]
    )
    
    screen_dir = get_package_share_directory('custom_guyi')
    included_screen_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(screen_dir, 'launch', 'gui.launch.py')),
                launch_arguments={'cam_topic': cam_topic, 'device': device}.items(),
            )
        ]
    )
    

    nodes = [
        included_cam_launch,
        ping1d_node,
        base_to_range,
        included_imu_launch,
        included_sonar_launch,
        included_screen_launch
    ]

    # This list should be in a params file
    topics = [
        'jetson_1/flir_camera/image_raw/compressed',
        'jetson_1/bar30/depth',
        'jetson_1/bar30/pressure',
        'jetson_1/bar30/temperature',
        'jetson_1/imagenex831l/range',
        'jetson_1/imu/data',
        'jetson_1/ekf/status',
        'jetson_1/imagenex831l/range_raw'
    ]

    return LaunchDescription(
        nodes + [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '--storage', 'sqlite3', '-o', ct_str] + topics,
                output='screen'
            )
        ]
    )
