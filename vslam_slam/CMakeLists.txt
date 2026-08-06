cmake_minimum_required(VERSION 3.8)
project(vslam_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.6 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ros2_orb_slam3 does not export its library as an ament target, so we point at
# its source tree for headers and its install tree for the .so. That location is
# fixed anyway: its common.cpp hardcodes packagePath = "ros2_ws/src/ros2_orb_slam3/".
set(ORB_SLAM3_SRC "$ENV{HOME}/ros2_ws/src/ros2_orb_slam3")
set(ORB_SLAM3_INSTALL "$ENV{HOME}/ros2_ws/install/ros2_orb_slam3/lib")

if(NOT EXISTS "${ORB_SLAM3_SRC}/orb_slam3/include/System.h")
  message(FATAL_ERROR
    "ORB-SLAM3 headers not found at ${ORB_SLAM3_SRC}. "
    "Clone Mechazo11/ros2_orb_slam3 (jazzy branch) into ~/ros2_ws/src and build it first.")
endif()

find_library(ORB_SLAM3_LIB orb_slam3_lib PATHS ${ORB_SLAM3_INSTALL} REQUIRED)
find_library(DBOW2_LIB DBoW2 PATHS ${ORB_SLAM3_INSTALL} REQUIRED)
find_library(G2O_LIB g2o PATHS ${ORB_SLAM3_INSTALL} REQUIRED)

add_executable(mono_slam_node src/mono_slam_node.cpp)

target_include_directories(mono_slam_node PRIVATE
  ${ORB_SLAM3_SRC}
  # KeyFrame.h includes "Thirdparty/DBoW2/DBoW2/BowVector.h", relative to here
  ${ORB_SLAM3_SRC}/orb_slam3
  ${ORB_SLAM3_SRC}/orb_slam3/include
  ${ORB_SLAM3_SRC}/orb_slam3/include/CameraModels
  ${ORB_SLAM3_SRC}/orb_slam3/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(mono_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros cv_bridge
)

target_link_libraries(mono_slam_node
  ${ORB_SLAM3_LIB}
  ${DBOW2_LIB}
  ${G2O_LIB}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

install(TARGETS mono_slam_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
