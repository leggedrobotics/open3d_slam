cmake_minimum_required(VERSION 3.5)
project(open3d_slam_utils)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)


set(SRC_FILES
        src/time.cpp
        src/Transform.cpp
        src/TransformInterpolationBuffer.cpp
        )

set(CATKIN_PACKAGE_DEPENDENCIES
        open3d_catkin
        )


find_package(Eigen3 REQUIRED)
find_package(Boost QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
    add_compile_options("${OpenMP_CXX_FLAGS}")
    add_definitions(-Dopen3d_slam_OPENMP_FOUND=${OpenMP_FOUND})
endif ()
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
)




include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
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

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
