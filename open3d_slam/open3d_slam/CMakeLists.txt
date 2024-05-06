cmake_minimum_required(VERSION 3.5)
project(open3d_slam)

# Set C++ standard and compiler optimizations
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

# Source files
set(SRC_FILES
  src/Mapper.cpp
  src/helpers.cpp
  src/time.cpp
  src/math.cpp
  src/Odometry.cpp
  src/Voxel.cpp
  src/croppers.cpp
  src/Submap.cpp
  src/SubmapCollection.cpp
  src/Transform.cpp
  src/TransformInterpolationBuffer.cpp
  src/AdjacencyMatrix.cpp
  src/PlaceRecognition.cpp
  src/OptimizationProblem.cpp
  src/SlamWrapper.cpp
  src/constraint_builders.cpp
  src/output.cpp
  src/MotionCompensation.cpp
  src/VoxelHashMap.cpp
  src/ScanToMapRegistration.cpp
  src/CloudRegistration.cpp
)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  add_definitions(-Dopen3d_slam_OPENMP_FOUND=${OpenMP_FOUND})
endif ()

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # Example, add more or modify as per your actual dependencies

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

# Library definition
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

# Specify library dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  Eigen3
  Boost
  OpenMP
)

# Export information
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost rclcpp sensor_msgs)

# Installation of nodes, libraries, etc.
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Generate executable if necessary
# add_executable(my_executable src/main.cpp)
# ament_target_dependencies(my_executable rclcpp)
# target_link_libraries(my_executable ${PROJECT_NAME})

ament_package()

