cmake_minimum_required(VERSION 3.5)
project(Lid_Cam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  open3d_conversions
  open3d_catkin
  eigen_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag
  cv_bridge
)
   


find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-Dopen3d_slam_OPENMP_FOUND=${OpenMP_FOUND})
endif()
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES
    yaml-cpp 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
  
)

add_library(${PROJECT_NAME}
  src/cam_proj.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${OpenMP_CXX_LIBRARIES}
)




add_executable(cam_proj
  src/cam_proj.cpp)
target_link_libraries(cam_proj
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

add_executable(mask
  src/mask.cpp)
target_link_libraries(mask
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

