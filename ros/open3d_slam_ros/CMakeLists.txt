cmake_minimum_required(VERSION 3.8)
project(open3d_slam_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

set(SRC_FILES
  src/helpers_ros.cpp
  src/SlamMapInitializer.cpp
  src/SlamWrapperRos.cpp
  src/OnlineRangeDataProcessorRos.cpp
  src/creators.cpp
  src/DataProcessorRos.cpp
  src/RosbagRangeDataProcessorRos.cpp
  src/Color.cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(open3d_slam_msgs REQUIRED)
find_package(open3d_slam_lua_io REQUIRED)
find_package(OpenMP QUIET)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(OpenMP_FOUND)
  target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
  target_compile_definitions(${PROJECT_NAME} PUBLIC open3d_slam_ros_OPENMP_FOUND=1)
endif()
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  interactive_markers
  rosbag2_cpp
  open3d_conversions
  open3d_slam
  open3d_slam_msgs
  open3d_slam_lua_io
)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${OpenMP_CXX_LIBRARIES})

add_executable(mapping_node src/mapping_node.cpp)
target_link_libraries(mapping_node ${PROJECT_NAME})
ament_target_dependencies(mapping_node rclcpp)

add_executable(adjacency_graph_example_node src/adjacency_graph_example_node.cpp)
target_link_libraries(adjacency_graph_example_node ${PROJECT_NAME})
ament_target_dependencies(adjacency_graph_example_node rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  mapping_node
  adjacency_graph_example_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch param rviz data DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  interactive_markers
  rosbag2_cpp
  open3d_conversions
  open3d_slam
  open3d_slam_msgs
  open3d_slam_lua_io
  yaml-cpp
)
ament_package()
