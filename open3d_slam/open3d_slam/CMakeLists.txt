cmake_minimum_required(VERSION 3.8)
project(open3d_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

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

find_package(ament_cmake REQUIRED)
find_package(open3d_catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP QUIET)

set(OPEN3D_SLAM_BOOST_LIBRARIES)
if(Boost_SERIALIZATION_LIBRARY_RELEASE)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_SERIALIZATION_LIBRARY_RELEASE}")
elseif(Boost_SERIALIZATION_LIBRARY)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_SERIALIZATION_LIBRARY}")
else()
  message(FATAL_ERROR "Boost serialization library not found")
endif()
if(Boost_FILESYSTEM_LIBRARY_RELEASE)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_FILESYSTEM_LIBRARY_RELEASE}")
elseif(Boost_FILESYSTEM_LIBRARY)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_FILESYSTEM_LIBRARY}")
else()
  message(FATAL_ERROR "Boost filesystem library not found")
endif()
if(Boost_SYSTEM_LIBRARY_RELEASE)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_SYSTEM_LIBRARY_RELEASE}")
elseif(Boost_SYSTEM_LIBRARY)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_SYSTEM_LIBRARY}")
else()
  message(FATAL_ERROR "Boost system library not found")
endif()
if(Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE}")
elseif(Boost_PROGRAM_OPTIONS_LIBRARY)
  list(APPEND OPEN3D_SLAM_BOOST_LIBRARIES "${Boost_PROGRAM_OPTIONS_LIBRARY}")
else()
  message(FATAL_ERROR "Boost program_options library not found")
endif()

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
if(OpenMP_FOUND)
  target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
  target_compile_definitions(${PROJECT_NAME} PUBLIC open3d_slam_OPENMP_FOUND=1)
endif()
ament_target_dependencies(${PROJECT_NAME} open3d_catkin)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${OPEN3D_SLAM_BOOST_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(open3d_catkin Boost yaml-cpp)
ament_package()
