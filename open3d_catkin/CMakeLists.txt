cmake_minimum_required(VERSION 3.5)
project(open3d_catkin)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
)


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
  INCLUDE_DIRS ${Open3D_INCLUDE_DIRS}
  LIBRARIES ${Open3D_LIBRARIES} 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Open3D 
)
  
