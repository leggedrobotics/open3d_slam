cmake_minimum_required(VERSION 3.16)
project(open3d_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(open3d_colcon REQUIRED)
find_package(OpenMP QUIET)

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

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    open3d_colcon::open3d_colcon
  PRIVATE
    ${OpenMP_CXX_LIBRARIES}
)

if(OpenMP_FOUND)
  target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE open3d_slam_OPENMP_FOUND=${OpenMP_FOUND})
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 open3d_colcon)
ament_export_include_directories(include)
ament_package()
