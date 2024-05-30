import os
import launch
import ament_index_python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():

    _MICROSTRAIN_LAUNCH_FILE = os.path.join(
        ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'),
        'launch', 'cv7_launch.py'
    )
    _CV7_PARAMS_FILE = os.path.join(
        ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'),
        'config', 'cv7', 'cv7.yml'
    )
    _EMPTY_PARAMS_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_driver'),
        'config', 'empty.yml'
    )

    camera_type = LaunchConfiguration('camera_type', default='blackfly_s')
    serial = LaunchConfiguration('serial', default="'20435009'")
    sonar = LaunchConfiguration('sonar', default='false')
    cam_topic = LaunchConfiguration('cam_topic', default='/debayer/image_raw/rgb')
    device = LaunchConfiguration('device', default="")

    def get_image():
        if LaunchConfigurationEquals(device, "jetson_2"):
            return ['jetson_2/image/compressed']
        else:
            return ['jetson_1/image/compressed']

    cam_dir = get_package_share_directory('spinnaker_camera_driver')
    included_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cam_dir, 'launch', 'driver_node.launch.py')),
        launch_arguments={'camera_type': camera_type, 'serial': serial}.items()
    )
    included_cam_launch_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('jetson_1'),
            included_cam_launch
        ]
    )

    # Depth 
    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        output="screen"
    )
    ping1d_node_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('jetson_1'),
            ping1d_node
        ]
    )

    base_to_range = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar30_link']
    )
    base_to_range_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('jetson_1'),
            base_to_range
        ]
    )

    included_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
        launch_arguments={'namespace': 'jetson_1'}.items()
    )
    included_imu_launch_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('jetson_1'),
            included_imu_launch
        ]
    )

    sonar_dir = get_package_share_directory('imagenex831l_ros2')
    included_sonar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sonar_dir, 'launch', 'sonar.launch.py')),
        launch_arguments={'sonar': sonar, 'device': device}.items()
    )
    
    screen_dir = get_package_share_directory('custom_guyi')
    included_screen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(screen_dir, 'launch', 'gui.launch.py')),
        launch_arguments={'cam_topic': cam_topic, 'device': device}.items()
    )
    
    debayer_node = Node(
        package='ros2_bringup',
        executable='debayer.py',
        name='debayer',
        output='screen',
        parameters=[{'cam_topic': cam_topic, 'device': device}]
    )

    nodes = [
        included_cam_launch_with_namespace,
        ping1d_node_with_namespace,
        base_to_range_with_namespace,
        included_imu_launch_with_namespace,
        included_sonar_launch,
        included_screen_launch,
        debayer_node
    ]

    topic1 = ['/jetson_1/image/compressed']
    topic2 = ['/jetson_1/bar30/depth']
    topic3 = ['/jetson_1/bar30/pressure']
    topic4 = ['/jetson_1/bar30/temperature']
    topic5 = ['/imagenex831l/range']
    topic6 = ['/jetson_1/imu/data']
    topic7 = ['/jetson_1/ekf/status']
    topic8 = ['/imagenex831l/range_raw']

    all_topics = topic1 + topic2 + topic3 + topic4 + topic5 + topic6 + topic7 + topic8

    return LaunchDescription(
        nodes + [ExecuteProcess(
            cmd = (['ros2', 'bag', 'record'] + all_topics),
            output = 'screen'
        )]
    )

