cmake_minimum_required(VERSION 3.18)
project(open3d_catkin)

find_package(ament_cmake REQUIRED)
find_package(Open3D QUIET)

add_library(${PROJECT_NAME} INTERFACE)

if(DEFINED ENV{Open3D_DIR})
  set(Open3D_DIR $ENV{Open3D_DIR})
  find_package(Open3D CONFIG REQUIRED)
  message(STATUS "Using Open3D from Open3D_DIR=${Open3D_DIR}")
  set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../include/")
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${Open3D_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${Open3D_INCLUDE_DIR_MAN}>
    $<INSTALL_INTERFACE:${Open3D_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${Open3D_INCLUDE_DIR_MAN}>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE ${Open3D_LIBRARIES})
elseif(Open3D_FOUND)
  message(STATUS "Using system Open3D")
  if(EXISTS "${Open3D_DIR}/../../../../include/")
    set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../../include/")
  else()
    set(Open3D_INCLUDE_DIR_MAN "${Open3D_DIR}/../../../include/")
  endif()

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${Open3D_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${Open3D_INCLUDE_DIR_MAN}>
    $<INSTALL_INTERFACE:${Open3D_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${Open3D_INCLUDE_DIR_MAN}>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE ${Open3D_LIBRARIES})
else()
  message(STATUS "Open3D not found, building it in-workspace")
  include(ExternalProject)

  set(OPEN3D_VENDOR_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/open3d_vendor")
  set(OPEN3D_VENDOR_INSTALL "${OPEN3D_VENDOR_PREFIX}/install")
  set(OPEN3D_VENDOR_LIBRARY_BUILD "${OPEN3D_VENDOR_INSTALL}/lib/libOpen3D.so")
  set(OPEN3D_VENDOR_LIBRARY_INSTALL "${CMAKE_INSTALL_PREFIX}/lib/libOpen3D.so")
  file(MAKE_DIRECTORY "${OPEN3D_VENDOR_INSTALL}/include")
  file(MAKE_DIRECTORY "${OPEN3D_VENDOR_INSTALL}/lib")
  file(TOUCH "${OPEN3D_VENDOR_LIBRARY_BUILD}")

  ExternalProject_Add(open3d
    GIT_REPOSITORY "https://github.com/isl-org/Open3D"
    GIT_TAG "v0.19.0"
    GIT_SHALLOW TRUE
    GIT_PROGRESS "true"
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true"
    PREFIX "${OPEN3D_VENDOR_PREFIX}"
    TMP_DIR "${OPEN3D_VENDOR_PREFIX}/tmp"
    STAMP_DIR "${OPEN3D_VENDOR_PREFIX}/stamp"
    DOWNLOAD_DIR "${OPEN3D_VENDOR_PREFIX}/download"
    SOURCE_DIR "${OPEN3D_VENDOR_PREFIX}/src/open3d"
    BINARY_DIR "${OPEN3D_VENDOR_PREFIX}/build"
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=ON
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
      -DCMAKE_CXX_FLAGS=-I/usr/include/minizip
      -DDEVELOPER_BUILD=OFF
      -DBUILD_GUI=OFF
      -DBUILD_WEBRTC=OFF
      -DBUILD_EXAMPLES=OFF
      -DBUILD_UNIT_TESTS=OFF
      -DBUILD_BENCHMARKS=OFF
      -DBUILD_PYTHON_MODULE=OFF
      -DBUILD_CUDA_MODULE=OFF
      -DBUILD_AZURE_KINECT=OFF
      -DBUILD_LIBREALSENSE=OFF
      -DUSE_SYSTEM_ASSIMP=ON
      -DUSE_SYSTEM_EIGEN3=ON
      -DUSE_SYSTEM_GLEW=ON
      -DUSE_SYSTEM_JPEG=ON
      -DUSE_SYSTEM_JSONCPP=ON
      -DUSE_SYSTEM_MSGPACK=ON
      -DUSE_SYSTEM_PNG=ON
      -DUSE_SYSTEM_QHULLCPP=ON
      -DUSE_SYSTEM_ZEROMQ=ON
      -DUSE_SYSTEM_BLAS=ON
      -DCMAKE_INSTALL_PREFIX=${OPEN3D_VENDOR_INSTALL}
  )

  add_dependencies(${PROJECT_NAME} open3d)
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${OPEN3D_VENDOR_INSTALL}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${OPEN3D_VENDOR_LIBRARY_BUILD}>
    $<INSTALL_INTERFACE:${OPEN3D_VENDOR_LIBRARY_INSTALL}>
  )

  install(DIRECTORY ${OPEN3D_VENDOR_INSTALL}/include/ DESTINATION include)
  install(DIRECTORY ${OPEN3D_VENDOR_INSTALL}/lib/ DESTINATION lib)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_dependencies(Open3D)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
