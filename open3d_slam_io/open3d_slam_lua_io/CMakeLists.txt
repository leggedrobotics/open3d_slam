cmake_minimum_required(VERSION 3.5)
project(open3d_slam_lua_io)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2)


set(SRC_FILES
  src/parameter_loaders.cpp
  src/LuaLoader.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  open3d_slam
  lua_parameter_dictionary
  rospack
)
  
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME} 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable( minimal_example
	src/minimal_example.cpp
)

target_link_libraries(minimal_example
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

