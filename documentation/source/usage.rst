Usage
=====

.. _installation:

Installation
------------

First, you need to install the open3Dslam package in your ROS catkin workspace: go to the github repository https://github.com/leggedrobotics/open3d_slam and follow the instructions.

Running
------------

First, download the example rosbags from here:
*drive folder with rosbags*

Place the rosbag in the data folder inside the open3d_slam_ros package

Launch with:

.. code-block:: console

   $ roslaunch open3d_slam_ros mapping_rosbag_puck.launch play_rate:=1.0 bag_filename:=*rosbag_filename*.bag cloud_topic:=/rslidar_points
