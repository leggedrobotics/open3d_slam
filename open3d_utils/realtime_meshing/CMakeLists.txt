cmake_minimum_required(VERSION 3.0.2)
project(realtime_meshing)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3)

SET(CATKIN_PACKAGES
        open3d_conversions
        open3d_catkin
        open3d_slam_utils
        eigen_conversions)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGES}
)


catkin_package(
        INCLUDE_DIRS include
        ${Eigen3_INCLUDE_DIRS}
        thirdparty/CDT/CDT/include
        thirdparty/ikd-Tree/ikd-Tree/
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGES}
        #  DEPENDS system_lib
)

include_directories(
        include
        thirdparty/CDT/CDT/include
        thirdparty/ikd-Tree/ikd-Tree/
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)
set(SOURCE_FILES
        src/Mesher.cpp
        src/MeshVoxel.cpp
        src/Plane.cpp
        src/helpers.cpp
        src/MeshMap.cpp)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp
        ${OpenMP_CXX_LIBRARIES}
        )