Usage
=====

We describe usage with ROS since it is very common way of using software for robotics. However, this package is
separated into *open3d_slam* (ROS independent) and *open3d_slam_ros* (ROS dependent). Hence, you can integrate
the non ROS dependent part with your own application.

Sample Data
-----------

You can download the example rosbags from here:
`rosbags <https://drive.google.com/drive/folders/1o7m91jBPBITZ9j9xpEniKz6IR3pwXAyC?usp=sharing>`__

Download the rosbags and place them in the *data* folder inside the *open3d_slam_ros* package.

Running
-------

*open3d_slam* can be run in two modes. Online mode you would typically use for operation on the robot which runs in
realtime. Offline mode you can use when building a map offline, this mode will read the rosbag and process all the
measurements as fast as possible.

Online
""""""
Online mode can be launched with ``mapping.launch.py``. This node subscribes to incoming pointclouds and starts estimating ego motion while
building a map.

.. note::

	Make sure you passed the correct path to the parameters file!!!! In general, different sensors will
	require different parameter tuning.
	
	
Launch with:

.. code-block:: console

   $ ros2 launch open3d_slam_ros mapping.launch.py \
       cloud_topic:=/rslidar_points \
       parameter_filename:=param_robosense_rs16.yaml

Launch arguments:

.. code-block:: text

   cloud_topic
   parameter_filename
   parameter_folder_path
   map_saving_folder
   num_accumulated_range_data

   
Offline
"""""""
Offline mode can be launched with ``mapping_rosbag.launch.py``. The Jazzy node reads the bag directly through ``rosbag2_cpp`` and processes
messages as fast as possible.

.. code-block:: console

   $ ros2 launch open3d_slam_ros mapping_rosbag.launch.py \
       rosbag_filepath:=/absolute/path/to/dataset \
       cloud_topic:=/rslidar_points \
       parameter_filename:=param_robosense_rs16.yaml
   
The offline launch shares all online arguments and adds one required launch argument:

.. code-block:: text

   rosbag_filepath

Happy mapping!
