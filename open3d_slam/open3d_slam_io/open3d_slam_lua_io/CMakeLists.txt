cmake_minimum_required(VERSION 3.5)
project(open3d_slam_lua_io)

# Set C++ standard and compiler optimizations
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

# Source files
set(SRC_FILES
  src/parameter_loaders.cpp
  src/LuaLoader.cpp
)

# Find required libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Assuming you're using standard ROS 2 components
find_package(ament_index_cpp REQUIRED)
find_package(lua_parameter_dictionary REQUIRED)  # You must create ROS 2 packages for these
find_package(open3d_slam REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system program_options)

set(GLOG_LIBRARY glog)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  lua_parameter_dictionary
  open3d_slam
  Eigen3
  Boost
)

# Installation of nodes, libraries, etc.
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Executable
add_executable(minimal_example
  src/minimal_example.cpp
)

target_link_libraries(minimal_example
  ${PROJECT_NAME}
  ${GLOG_LIBRARY}
)

ament_target_dependencies(minimal_example
  rclcpp
  lua_parameter_dictionary
  open3d_slam
  Eigen3
  Boost
)

# Install the library and executable
install(TARGETS minimal_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp lua_parameter_dictionary open3d_slam)

# Setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize ament package
ament_package()

