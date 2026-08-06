cmake_minimum_required(VERSION 3.8)
project(vslam_description)

find_package(ament_cmake REQUIRED)

# meshes/ is required: gz_slim_tb3.sdf.xacro references
# package://vslam_description/meshes/burger_deck1.stl, and gz resolves that
# against the install tree. Omitting it renders the robot body invisible.
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
