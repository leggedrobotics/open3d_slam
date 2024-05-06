cmake_minimum_required(VERSION 3.5.1)
project(lua_parameter_dictionary)

set(CMAKE_CXX_STANDARD 17)
# add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # Assuming usage of rclcpp
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Lua REQUIRED)

# Set Boost and glog manually if needed
set(GLOG_LIBRARY glog)
set(BOOST_COMPONENTS iostreams)

# Define include directories
include_directories(
  include
  ${LUA_INCLUDE_DIR}
)

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/lua_parameter_dictionary.cc
  src/configuration_file_resolver.cc
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${LUA_LIBRARIES}
  ${GLOG_LIBRARY}
  gflags
)

# Ament target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  Boost
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executable
add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})

# Install targets, headers, and libraries
install(TARGETS example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(rclcpp Boost)
ament_export_include_directories(include ${LUA_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME} ${LUA_LIBRARIES} ${GLOG_LIBRARY})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add gtest based cpp test target and link libraries
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_lua.cpp
    test/lua_parameter_dictionary_test.cc
  )
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

# Finalize ament package
ament_package()

