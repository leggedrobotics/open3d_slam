cmake_minimum_required(VERSION 3.5)
project(open3d_rosbag_mapper_mesher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

# Source Files in this Project
set(SRC_FILES
        src/lib/RosbagMapperRos.cpp
        src/lib/MapMesher.cpp
)

# Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        roslib
        open3d_conversions
        open3d_slam
        open3d_slam_ros
        eigen_conversions
        tf
        tf2
        tf2_msgs
        tf2_ros
        tf2_geometry_msgs
        rosbag
        interactive_markers
        open3d_slam_lua_io
)

# CMake Dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
    add_compile_options("${OpenMP_CXX_FLAGS}")
    add_definitions(-Dopen3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
endif ()
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Main Catkin Package
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

# Include Directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${OpenMP_CXX_INCLUDE_DIRS}
)

# Main Library
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

# Main Node
add_executable(${PROJECT_NAME}_node
        src/main_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
