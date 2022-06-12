Usage
=====

Sample Data
-------

You can download the example rosbags from here:
`documentation <https://drive.google.com/drive/folders/1JE1WLFK60cocSiK3-BvjcaFZaLvxGsg-?usp=sharing>`__

Running
-------

*open3D_slam* can be run in two modes. Online mode you would typically use for operation on the robot which runs in
realtime. Offline mode you can use when building a map offline, this mode will read the rosbag and process all the
measurements as fast as possible.

Online
""""""


Place the rosbag in the data folder inside the open3d_slam_ros package

Launch with:

.. code-block:: console

   $ roslaunch open3d_slam_ros mapping_rosbag_puck.launch play_rate:=1.0 bag_filename:=*rosbag_filename*.bag cloud_topic:=/rslidar_points

   
Offline
"""""""