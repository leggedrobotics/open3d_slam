cmake_minimum_required(VERSION 3.5)
project(m545_volumetric_mapping)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  open3d_conversions
)

find_package(Eigen3 REQUIRED)


if(DEFINED ENV{Open3D_DIR})
  set(Open3D_DIR ENV{Open3D_DIR})
  find_package(Open3D CONFIG REQUIRED)
else()
  find_package(Open3D REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

add_executable(mapping_node
  src/mapping_node.cpp)
target_link_libraries(mapping_node
    ${catkin_LIBRARIES}
)
  
