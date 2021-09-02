cmake_minimum_required(VERSION 3.18)
project(m545_volumetric_mapping)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
)

set(Open3D_DIR "/home/jelavice/Programs/open3d_install/lib/cmake/Open3D")
find_package(Open3D CONFIG REQUIRED)

find_package( catkin REQUIRED
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

message(WARNING "include dir: ${Open3D_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS})

add_executable(mapping_node
  src/mapping_node.cpp)
target_link_libraries(mapping_node
  ${Open3D_LIBRARIES})
