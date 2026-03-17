cmake_minimum_required(VERSION 3.8)
project(open3d_conversions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(open3d_catkin REQUIRED)
find_package(open3d_slam_msgs REQUIRED)

add_library(${PROJECT_NAME} src/open3d_conversions.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  open3d_catkin
  open3d_slam_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  if(TARGET test_open3d_conversions)
    target_link_libraries(test_open3d_conversions ${PROJECT_NAME})
    ament_target_dependencies(test_open3d_conversions sensor_msgs open3d_slam_msgs)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs open3d_catkin open3d_slam_msgs)
ament_package()
