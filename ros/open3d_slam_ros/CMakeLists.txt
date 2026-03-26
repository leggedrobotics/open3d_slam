cmake_minimum_required(VERSION 3.16)
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
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(open3d_slam_msgs REQUIRED)
find_package(open3d_slam_yaml_io REQUIRED)
find_package(OpenMP QUIET)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  open3d_conversions::open3d_conversions
  open3d_slam::open3d_slam
  open3d_slam_yaml_io::open3d_slam_yaml_io
  ${OpenMP_CXX_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
  interactive_markers
  nav_msgs
  open3d_slam_msgs
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

if(OpenMP_FOUND)
  target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE open3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
endif()

add_executable(mapping_node src/mapping_node.cpp)
target_link_libraries(mapping_node ${PROJECT_NAME})

add_executable(adjacency_graph_example_node src/adjacency_graph_example_node.cpp)
target_link_libraries(adjacency_graph_example_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} mapping_node adjacency_graph_example_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch param rviz data DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_runtime_contracts test/test_runtime_contracts.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  interactive_markers
  nav_msgs
  open3d_conversions
  open3d_slam
  open3d_slam_msgs
  open3d_slam_yaml_io
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
ament_export_include_directories(include)
ament_package()
