cmake_minimum_required(VERSION 3.8)
project(open3d_slam_lua_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

set(SRC_FILES
  src/parameter_loaders.cpp
  src/LuaLoader.cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(open3d_slam REQUIRED)
find_package(lua_parameter_dictionary REQUIRED)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  open3d_slam
  lua_parameter_dictionary
)

add_executable(minimal_example src/minimal_example.cpp)
target_link_libraries(minimal_example ${PROJECT_NAME})
ament_target_dependencies(minimal_example ament_index_cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS minimal_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY example_param/ DESTINATION share/${PROJECT_NAME}/example_param)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(open3d_slam lua_parameter_dictionary)
ament_package()
