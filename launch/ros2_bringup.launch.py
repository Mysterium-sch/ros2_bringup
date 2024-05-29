import os
import launch
import ament_index_python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():

    _MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'launch', 'cv7_launch.py')
    _CV7_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_examples'), 'config', 'cv7', 'cv7.yml')
    _EMPTY_PARAMS_FILE = os.path.join(get_package_share_directory('microstrain_inertial_driver'),'config','empty.yml')

    camera_type = LaunchConfiguration('camera_type', default='blackfly_s')
    serial = LaunchConfiguration('serial', default="'20435009'")
    sonar = LaunchConfiguration('sonar', default='false')
    cam_topic = LaunchConfiguration('cam_topic', default='/debayer/image_raw/rgb')
    device = LaunchConfiguration('device', default="")
    imu = LaunchConfiguration('imu', default="cv7")

    def get_params_file():
        if LaunchConfigurationEquals(imu, "empty"):
            return _EMPTY_PARAMS_FILE
        else:
            return _CV7_PARAMS_FILE

    cam_dir = get_package_share_directory('spinnaker_camera_driver')
    included_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cam_dir, 'launch', 'driver_node.launch.py')),
        launch_arguments={'camera_type': camera_type, 'serial': serial}.items()
    )

    #Depth 
    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        output="screen"
    )

    base_to_range = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    included_imu_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
     # launch_arguments={
     #   'configure': 'true',
     #   'activate': 'true',
     #   'params_file': get_params_file(),
     #   'namespace': '/',
     # }.items(),
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
        included_cam_launch,
        ping1d_node,
        base_to_range,
        included_imu_launch,
        included_sonar_launch,
        included_screen_launch,
        debayer_node
    ]

    topic1 = ['/flir_camera/image_raw']
    topic2 = ['/bar30/depth']
    topic3 = ['/imagenex831l/range']
    topic4 = ['/imu/data']
    topic5 = ['/ekf/status']

    all_topics = topic1 + topic2 + topic3 + topic4 + topic5

    return LaunchDescription(
        nodes + [ExecuteProcess(
            cmd = (['ros2', 'bag', 'record'] + all_topics),
            output = 'screen' )
    ])
