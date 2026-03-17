cmake_minimum_required(VERSION 3.8)
project(lua_parameter_dictionary)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(GLOG_LIBRARY glog)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Lua REQUIRED)

add_library(${PROJECT_NAME}
  src/lua_parameter_dictionary.cc
  src/configuration_file_resolver.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${LUA_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${LUA_LIBRARIES}
  ${GLOG_LIBRARY}
  gflags
)

add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
ament_target_dependencies(example ament_index_cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_lua.cpp
    test/lua_parameter_dictionary_test.cc
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_include_directories(${PROJECT_NAME}-test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
