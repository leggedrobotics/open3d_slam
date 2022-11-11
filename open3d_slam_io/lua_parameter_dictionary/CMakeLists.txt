cmake_minimum_required(VERSION 3.5.1)

project(lua_parameter_dictionary)

set(CMAKE_CXX_STANDARD 14)
#add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(GLOG_LIBRARY glog)
set(BOOST_COMPONENTS iostreams)
set(CATKIN_PACKAGE_DEPENDENCIES  
  rospack
  roslib
)


find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Lua REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${LUA_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
    ${LUA_LIBRARIES}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${LUA_INCLUDE_DIR}
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/lua_parameter_dictionary.cc
   src/configuration_file_resolver.cc
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LUA_LIBRARIES}
  ${GLOG_LIBRARY}
  gflags
)

add_executable( example
	src/example.cpp
)

target_link_libraries(example
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN " *.h"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_lua.cpp
    test/lua_parameter_dictionary_test.cc
  )
  target_include_directories(${PROJECT_NAME}-test PRIVATE
    include
  )
  target_include_directories(${PROJECT_NAME}-test SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        ${PROJECT_NAME}-test
    )
  endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)