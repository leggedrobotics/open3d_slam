cmake_minimum_required(VERSION 3.16)
project(open3d_colcon)

find_package(ament_cmake REQUIRED)
find_package(Open3D REQUIRED CONFIG)

set(OPEN3D_LIBRARY_DIR "")
if(TARGET Open3D::Open3D)
  foreach(open3d_imported_property
      IMPORTED_LOCATION
      IMPORTED_LOCATION_RELEASE
      IMPORTED_LOCATION_RELWITHDEBINFO
      IMPORTED_LOCATION_DEBUG
      IMPORTED_IMPLIB
      IMPORTED_IMPLIB_RELEASE
      IMPORTED_IMPLIB_RELWITHDEBINFO
      IMPORTED_IMPLIB_DEBUG)
    get_target_property(open3d_imported_location Open3D::Open3D ${open3d_imported_property})
    if(open3d_imported_location AND NOT open3d_imported_location MATCHES "-NOTFOUND$")
      get_filename_component(OPEN3D_LIBRARY_DIR "${open3d_imported_location}" DIRECTORY)
      break()
    endif()
  endforeach()
endif()

if(NOT OPEN3D_LIBRARY_DIR)
  if(DEFINED Open3D_LIBRARY_DIRS AND Open3D_LIBRARY_DIRS)
    list(GET Open3D_LIBRARY_DIRS 0 OPEN3D_LIBRARY_DIR)
  else()
    foreach(open3d_library IN LISTS Open3D_LIBRARIES)
      if(IS_ABSOLUTE "${open3d_library}")
        get_filename_component(OPEN3D_LIBRARY_DIR "${open3d_library}" DIRECTORY)
        break()
      endif()
    endforeach()
  endif()
endif()

add_library(${PROJECT_NAME} INTERFACE)
if(TARGET Open3D::Open3D)
  target_link_libraries(${PROJECT_NAME} INTERFACE Open3D::Open3D)
else()
  target_include_directories(${PROJECT_NAME} INTERFACE ${Open3D_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} INTERFACE ${Open3D_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if(OPEN3D_LIBRARY_DIR)
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/open3d_library_path.dsv.in")
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Open3D)
ament_package()
