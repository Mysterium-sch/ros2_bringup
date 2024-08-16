cmake_minimum_required(VERSION 3.5)
project(ros2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_python_install_package(src)

install(PROGRAMS 
  src/debayer.py
  src/rosbag.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
