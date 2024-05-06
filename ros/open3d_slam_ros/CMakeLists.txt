cmake_minimum_required(VERSION 3.5)
project(open3d_slam_ros)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-g)

# Source files
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

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(OpenMP QUIET)
find_package(visualization_msgs REQUIRED)
find_package(Open3D REQUIRED) # Ensure Open3D or a wrapper is available as a ROS 2 package
find_package(open3d_slam REQUIRED)
find_package(open3d_slam_lua_io REQUIRED)
find_package(open3d_slam_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

# Define library
add_library(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rosbag2_cpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  Eigen3
  Boost
  Open3D
  open3d_slam
  open3d_slam_lua_io
  open3d_slam_msgs
  open3d_conversions
  interactive_markers
  visualization_msgs
)

if(OpenMP_CXX_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC open3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
  target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_LIBRARIES})
endif()

# Installation of nodes, libraries, etc.
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Executables
add_executable(mapping_node src/mapping_node.cpp)
target_link_libraries(mapping_node ${PROJECT_NAME})

add_executable(adjacency_graph_example_node src/adjacency_graph_example_node.cpp)
target_link_libraries(adjacency_graph_example_node ${PROJECT_NAME})

# Install the library and executables
install(TARGETS mapping_node adjacency_graph_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rosbag2_cpp std_msgs std_srvs geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs sensor_msgs Eigen3 Open3D open3d_slam open3d_slam_lua_io open3d_slam_msgs open3d_conversions interactive_markers visualization_msgs)

# Setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize ament package
ament_package()

