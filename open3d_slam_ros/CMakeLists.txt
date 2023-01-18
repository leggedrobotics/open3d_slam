cmake_minimum_required(VERSION 3.5)
project(open3d_slam_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)


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

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  open3d_conversions
  open3d_slam
  eigen_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag
  interactive_markers
  open3d_slam_lua_io
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-Dopen3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
endif()
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES
    yaml-cpp
    ${PROJECT_NAME} 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${OpenMP_CXX_LIBRARIES}
)


add_executable(mapping_node
  src/mapping_node.cpp)
target_link_libraries(mapping_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

add_executable(adjacency_graph_example_node
  src/adjacency_graph_example_node.cpp
)

target_link_libraries(adjacency_graph_example_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


