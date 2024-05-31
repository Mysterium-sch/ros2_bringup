import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def ensure_required_arguments(context, *args, **kwargs):
    required_args = ['namespace', 'camera_type', 'serial', 'sonar', 'cam_topic']
    for arg in required_args:
        if not context.launch_configurations.get(arg):
            raise RuntimeError(f"The '{arg}' argument is required.")
    return []

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    camera_type = LaunchConfiguration('camera_type')
    serial = LaunchConfiguration('serial')
    sonar = LaunchConfiguration('sonar')
    cam_topic = LaunchConfiguration('cam_topic')

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
                launch_arguments={'camera_type': camera_type, 'serial': serial}.items()
            )
        ]
    )

    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        namespace=namespace,
        output="screen"
    )

    # TODO: Fix Me
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
                launch_arguments={'sonar': sonar, 'device': namespace, 'config': config}.items()
            )
        ]
    )
    
    screen_dir = get_package_share_directory('custom_guyi')
    included_screen_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(screen_dir, 'launch', 'gui.launch.py')),
                launch_arguments={'cam_topic': cam_topic, 'device': namespace}.items(),
            )
        ]
    )
    
    debayer_node = Node(
        package='ros2_bringup',
        executable='debayer.py',
        name='debayer',
        namespace=namespace,
        output='screen',
        parameters=[{'cam_topic': cam_topic, 'device': namespace}]
    )

    nodes = [
        included_cam_launch,
        ping1d_node,
        base_to_range,
        included_imu_launch,
        included_sonar_launch,
        included_screen_launch,
        debayer_node
    ]

    # This list should be in a params file
    topics = [
        'jetson_2/image/compressed',
        'jetson_2/bar30/depth',
        'jetson_2/bar30/pressure',
        'jetson_2/bar30/temperature',
        'jetson_2/imagenex831l/range',
        'jetson_2/imu/data',
        'jetson_2/ekf/status',
        'jetson_2/imagenex831l/range_raw'
    ]

    return LaunchDescription(
        [
            OpaqueFunction(function=ensure_required_arguments)
        ] + nodes + [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record'] + topics,
                output='screen'
            )
        ]
    )
