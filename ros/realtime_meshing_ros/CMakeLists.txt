cmake_minimum_required(VERSION 3.0.2)
project(realtime_meshing_ros)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
        roscpp
        open3d_catkin
        open3d_slam_utils
        open3d_conversions
        realtime_meshing)


catkin_package(
        INCLUDE_DIRS include
        ${Eigen3_INCLUDE_DIRS}
        #  LIBRARIES realtime_meshing
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)
set(SOURCE_FILES
        src/RealtimeMeshingRos.cpp
        src/realtime_meshing_node.cpp
)

add_executable(realtime_meshing_node ${SOURCE_FILES})

target_link_libraries(realtime_meshing_node
        ${catkin_LIBRARIES}
        yaml-cpp
        )