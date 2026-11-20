cmake_minimum_required(VERSION 3.16)
project(panthera_dual_description NONE)
# Optional ROS 2 packaging only; the SDK itself loads files from the source tree.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY urdf srdf meshes config DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
