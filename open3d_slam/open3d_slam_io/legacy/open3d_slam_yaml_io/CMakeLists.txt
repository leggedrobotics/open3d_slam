cmake_minimum_required(VERSION 3.8)
project(open3d_slam_yaml_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} src/parameter_loaders.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} open3d_slam)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(open3d_slam)
ament_package()
