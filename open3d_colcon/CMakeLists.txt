cmake_minimum_required(VERSION 3.16)
project(open3d_colcon)

find_package(ament_cmake REQUIRED)
find_package(Open3D REQUIRED CONFIG)

add_library(${PROJECT_NAME} INTERFACE)
if(TARGET Open3D::Open3D)
  target_link_libraries(${PROJECT_NAME} INTERFACE Open3D::Open3D)
else()
  target_include_directories(${PROJECT_NAME} INTERFACE ${Open3D_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} INTERFACE ${Open3D_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Open3D)
ament_package()
