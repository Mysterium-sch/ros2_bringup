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


	included_cam_launch = IncludeLaunchDescription(
	      PythonLaunchDescriptionSource([os.path.join(
		 get_package_share_directory('spinnaker_camera_driver'), 'launch'),
		 '/driver_node.launch.py']),
	          launch_arguments={'camera_type': camera_type, 'serial': serial, 'computer_brightness': 'true'}.items()
    )
		        
	depth_dir = get_package_share_directory('ms5837_bar_ros')


	included_depth_launch = IncludeLaunchDescription(
	      PythonLaunchDescriptionSource([os.path.join(
		 get_package_share_directory('ms5837_bar_ros'), 'launch'),
		 '/bar30.launch.py'])
	      )
	      
	imu_dir = get_package_share_directory('microstrain_inertial_driver')
	
	included_imu_launch = IncludeLaunchDescription(
	      PythonLaunchDescriptionSource([os.path.join(
		 get_package_share_directory('microstrain_inertial_driver'), 'launch'),
		 '/microstrain_launch.py'])
	      )
       
	return LaunchDescription([
		included_cam_launch,
		included_depth_launch,
		included_imu_launch,
		Node(
        package='ros2_bringup',
        namespace='debayer',
	    executable='debayer.py',
        name='debayer'
        ),
	])
                

