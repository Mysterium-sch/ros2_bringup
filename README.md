# Bringup node

## Instructions
To launch:

### Jetson_1:

``ros2 launch ros2_bringup stereorig.launch.py namespace:=jetson_1 camera_type:=blackfly_s serial:=? sonar:=false cam_topic:=``

### Jetson_2:

``ros2 launch ros2_bringup stereorig.launch.py namespace:=? camera_type:=? serial:=? sonar:=? cam_topic:=?``