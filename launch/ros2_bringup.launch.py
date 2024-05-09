import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    cam_dir = get_package_share_directory('spinnaker_camera_driver')
    camera_type = LaunchConfiguration('camera_type', default='blackfly_s')
    serial = LaunchConfiguration('serial', default="'20435009'")
    sonar = LaunchConfiguration('sonar', default='false')

    included_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            cam_dir, 'launch'), '/driver_node.launch.py']),
        launch_arguments={'camera_type': camera_type, 'serial': serial, 'computer_brightness': computer_brightness}.items()
    )
        
    depth_dir = get_package_share_directory('ms5837_bar_ros')
    included_depth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            depth_dir, 'launch'), '/bar30.launch.py'])
    )
    
    imu_dir = get_package_share_directory('microstrain_inertial_driver')
    included_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            imu_dir, 'launch'), '/microstrain_launch.py'])
    )

    if sonar == 'true':
        sonar_dir = get_package_share_directory('imagenex831l_ros2')
        included_sonar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                sonar_dir, 'launch'), '/sonar.launch.py'])
        )
        nodes = [included_cam_launch, included_depth_launch, included_imu_launch, included_sonar_launch]
    else:
        nodes = [included_cam_launch, included_depth_launch, included_imu_launch]

    return LaunchDescription(nodes)



