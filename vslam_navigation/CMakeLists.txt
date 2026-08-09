cmake_minimum_required(VERSION 3.8)
project(vslam_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# chmod +x in the source tree matters: --symlink-install only symlinks, so a
# non-executable source gives "executable not found on the libexec directory"
install(PROGRAMS
  scripts/goal_relay.py
  scripts/send_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
