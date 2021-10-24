cmake_minimum_required(VERSION 3.5)
project(m545_volumetric_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)


set(SRC_FILES
  src/Parameters.cpp
  src/Mapper.cpp
  src/helpers.cpp
  src/helpers_ros.cpp
  src/time.cpp
  src/math.cpp
  src/Mesher.cpp
  src/Odometry.cpp
  src/Voxel.cpp
  src/croppers.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  open3d_conversions
  open3d_catkin
  eigen_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)
   


find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DM545_VOLUMETRIC_MAPPING_OPENMP_FOUND=${OpenMP_FOUND})
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

