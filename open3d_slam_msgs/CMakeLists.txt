cmake_minimum_required(VERSION 3.5.1)
project(open3d_slam_msgs)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Vertices.msg
  PolygonMesh.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveMap.srv
  SaveSubmaps.srv 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES
    CATKIN_DEPENDS
        sensor_msgs
        message_runtime
    #DEPENDS
)

