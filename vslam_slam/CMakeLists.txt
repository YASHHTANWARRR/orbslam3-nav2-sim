cmake_minimum_required(VERSION 3.8)
project(vslam_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Installed into lib/ so launch_ros finds it as an executable.
install(
  PROGRAMS scripts/gz_camera_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
