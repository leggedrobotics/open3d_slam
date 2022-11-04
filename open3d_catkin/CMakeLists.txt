cmake_minimum_required(VERSION 3.16)
project(open3d_catkin)

find_package(catkin REQUIRED)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Check whether open3d can be found
find_package(Open3D QUIET)

# Color
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
  set(Magenta     "${Esc}[35m")
endif()

# 1 - Check whether variable is set -----------------------------------------------------------
if(DEFINED ENV{Open3D_DIR})
  set(Open3D_DIR ENV{Open3D_DIR})
  find_package(Open3D CONFIG REQUIRED)
  message("${BoldMagenta}INFO: Found manually set path to Open3D. Using version located at $ENV{Open3D_DIR}.${ColourReset}")
  set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../include/")
  message("Open3D include path: ${Open3D_INCLUDE_DIRS} , ${Open3D_INCLUDE_DIR_MAN}")
  message("Open3D libs: ${Open3D_LIBRARIES}")
  catkin_package(
  INCLUDE_DIRS 
    ${Open3D_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIR_MAN}
  LIBRARIES 
    ${Open3D_LIBRARIES} 
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Open3D 
  )
# 2 - Check whether open3d is installed -------------------------------------------------------
elseif(Open3D_FOUND)
  message("${BoldMagenta}INFO: Found Open3D.${ColourReset}")
  
  # Path differs whether installed from PPA or (locally) from source
  if(EXISTS "${Open3D_DIR}/../../../../include/")
    set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../../include/")
  else()
    set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../include/")
  endif()
  
  message("Open3D libraries: ${Open3D_LIBRARIES}")
  message("Open3D include path: ${Open3D_INCLUDE_DIRS} , ${Open3D_INCLUDE_DIR_MAN}")
  
  
  find_package(fmt QUIET)
  if(fmt_FOUND)
  	message("Linking fmt into open3d_catkin")
  	catkin_package(
      INCLUDE_DIRS 
        ${Open3D_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIR_MAN}
      LIBRARIES 
        ${Open3D_LIBRARIES}
        fmt::fmt
      CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
      DEPENDS
        Open3D 
    )
  else()
  	message("fmt not found. If you get linking errors try installing libfmt with sudo apt install libfmt-dev")
  	catkin_package(
      INCLUDE_DIRS 
        ${Open3D_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIR_MAN}
      LIBRARIES 
        ${Open3D_LIBRARIES}
      CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
      DEPENDS
        Open3D 
    )  	 
  endif()


# 3 - Compile in catkin workspace -------------------------------------------------------------
else()
  message("${BoldMagenta}INFO: Neither variable Open3d_DIR is set, nor could Open3D be found, hence compiling in workspace.${ColourReset}")
  message("${BoldMagenta}INFO: This automatic compilation from source requires CMAKE>3.18.${ColourReset}")

  # Newer version of CMake is required
  cmake_minimum_required(VERSION 3.18)

  # Catkinization of open3D
  include(ExternalProject)
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

  ExternalProject_Add(open3d
    GIT_REPOSITORY "https://github.com/isl-org/Open3D"
    GIT_TAG "v0.15.1"
    GIT_SUBMODULES_RECURSE "true"
    GIT_PROGRESS "true"
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true"
    PREFIX "${CMAKE_SOURCE_DIR}"
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/open3d"
    BINARY_DIR "${CMAKE_SOURCE_DIR}/tmp"
    CMAKE_ARGS 
         -DBUILD_SHARED_LIBS=ON
         -DCMAKE_BUILD_TYPE=RelWithDebInfo
         -DUSE_SYSTEM_EIGEN3=OFF
         -DGLIBCXX_USE_CXX11_ABI=ON
         -DBUILD_PYTHON_MODULE=OFF
         -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
    INSTALL_COMMAND make install && bash "-c" "cp -r ${CMAKE_SOURCE_DIR}/tmp/fmt/include/ ${CATKIN_DEVEL_PREFIX} && cp ${CMAKE_SOURCE_DIR}/tmp/fmt/lib/*.a ${CATKIN_DEVEL_PREFIX}/lib/"
  )

  catkin_package(
    INCLUDE_DIRS 
      ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES
      ${CATKIN_DEVEL_PREFIX}/lib/libOpen3D.so
      ${CATKIN_DEVEL_PREFIX}/lib/libfmt.a
    CATKIN_DEPENDS
  )

  #############
  ## INSTALL ##
  #############

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
    DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  )
  
  if(${MAKE_LOCAL_INSTALL})
    message("${BoldMagenta}INFO: Making a local install.${ColourReset}")
  	add_custom_command (OUTPUT run_install_script___OK 
                        COMMAND ${CMAKE_SOURCE_DIR}/local_install.sh "${CMAKE_SOURCE_DIR}"
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/)         
  	add_custom_target(
  	  RunInstallScript ALL
  	  DEPENDS run_install_script___OK)
  endif()

endif()
