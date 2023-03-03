cmake_minimum_required(VERSION 3.5.1)
project(open3d_slam_msgs_ros2)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces( ${PROJECT_NAME}
  msg/Vertices.msg
  msg/PolygonMesh.msg
  srv/SaveMap.srv
  srv/SaveSubmaps.srv
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
