# ros2_bringup

This ROS2 package serves two major purposes: 
1. Launch all the required packages for the stereo rig.
2. Launch a debayer node to successfully convert the images produced from the `flir_camera_driver` node to both regular RGB images and compressed RGB images.

The package consists of two launch files, one for Jetson 1 and one for Jetson 2. The node requires five parameters:
- **namespace**
- **camera_type**
- **serial**
- **sonar**
- **cam_topic**

The `camera_type` and `serial` parameters are fed into the `flir_camera_driver` package and are required for the camera to properly connect to the rest of the system. The `sonar` parameter indicates whether the system should launch the sonar node. The `cam_topic` parameter specifies which topic the GUI should subscribe to in order to display the image. The `namespace` parameter declares the namespace for the system.

The `ros2_bringup` package also starts the rosbag and records all the necessary topics.

## Instructions
To launch:
1. Build all required packages:
   - **custom_guyi**
   - **flir_camera_driver**
   - **imagenex831l_ros2**
   - **microstrain_inertial**
   - **ms5837_bar_ros**
   - **ros2_bringup**
2. Source `install/setup.bash`
3. Run the appropriate launch command based on the device

#### Jetson_1:

``ros2 launch ros2_bringup stereorig_1.launch.py serial:="'21502646'" sonar:='false' namespace:='jetson_1' cam_topic:="debayer/image_raw/rgb" camera_type:="blackfly_s"``

#### Jetson_2:

``ros2 launch ros2_bringup stereorig_2.launch.py serial:="'21387972'" sonar:='true'  namespace:='jetson_2' camera_type:="blackfly_s" cam_topic:="debayer/image_raw/rgb"``
