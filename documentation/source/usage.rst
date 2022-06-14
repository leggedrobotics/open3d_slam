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
Online mode can be launched with *mapping.launch* launch file. This node, will subscribe to incoming pointclouds (range sensor data)
and start estimating ego motion and build a map. 

.. note::

	Make sure you passed the correct path to the parameters file!!!! In general, different sensors will
	require different parameter tuning.
	
	
Launch with:

.. code-block:: console

   $ roslaunch open3d_slam_ros mapping.launch

You can take a look at the launch file arguments below:

.. note::

   This node also supports offline processing so don't get confused with rosbag parameters.


.. code-block:: xml 

	<arg name="launch_prefix" default="" doc="gdb -ex run --args"/>
	Launch prefix. Can be used to run in debug mode.

	<arg name="launch_rviz" default="false" />
	Whether to launch rviz or not.
	 	
	<arg name="cloud_topic" default="/rslidar_points" />
	Rostopic name for the incoming pointcloud stream.
	
	<arg name="parameter_filename" default="params_robosense_rs16.yaml"/>
	Name of the parameter file for the algorithm.

	<arg name="parameter_folder_path" default="$(find open3d_slam_ros)/param/"/>
	Path to the folder containing parameter file.

	<arg name="map_saving_folder" default="$(find open3d_slam_ros)/data/maps/"/>
	Folder where to save maps at the end of the mission or when rosservice is called.
	If the folder does not exist it tries to create it.
	
	<arg name="num_accumulated_range_data" default="1"/>
	Number of accumulated range data. Useful if you are streaming small packets and not full
	pointclouds, then the node will batch them and send them to processing.
	
	<arg name="is_read_from_rosbag" default="false"/>
	See offline mode below.
	
	<arg name="rosbag_filepath" default=""/>
	Full path to the rosbag for the case when *is_read_from_rosbag* argument is set to true.

	<arg name="use_sim_time" default="false"/>
	See offline mode below.

   
Offline
"""""""

Offline mode can be launched with *mapping_rosbag.launch* launch file. This launch includes the *mapping.launch* and adds some extra features.
Same as online mode, the offline mode will start estimating ego motion and build a map. In addition, offline mode will launch the rosbag player 
if needed. There are two ways of using the offline node 

The recommended way of building maps offline is to iterate through rosbag and process range measurements as fast as possible, 
then set the read_from_rosbag argument to true:

.. code-block:: console

   $ roslaunch open3d_slam_ros mapping_rosbag.launch is_read_from_rosbag:=true
   

Reading from rosbag is very useful for building maps offline since the processing can be performed much faster. For example the *wheeled_robot_large_outdoor_area.bag*
is 663 seconds long, but if we iterate through rosbag we process it in 220 seconds. *legged_robot_large_building.bag* is 3799 seconds long, however we can process it in 484 seconds. 

Another important aspect of iterating through rosbag is that we can force loop closures for the  last submap, thus
potentially correcting for the drift. Note that we can not do this in online operation (or when using rosbag player) since
we do not know when the mapping session is finished. 


If you want to play a rosbag with rosbag player then launch with:

.. code-block:: console

   $ roslaunch open3d_slam_ros mapping_rosbag.launch

*mapping_rosbag.launch* shares a lot of arguments with *mapping.launch* so below we list the ones that are different:

.. code-block:: xml 

	<arg name="is_read_from_rosbag" default="false"/>
	Whether to iterate through rosbag (if set to true) or launch 
	the rosbag player (is set to to false).
	
	<arg name="bag_filename" default="wheeled_robot_large_outdoor_area.bag" /> 
	Name of the rosbag file.
	
	<arg name="bag_folder_path" default="$(find open3d_slam_ros)/data/"/>
	Path to the folder containing the rosbag file.
	
	<arg name="play_delay" default="0.4" />
	Delay in playing rosbag after the rosbag player has started.
	
	<arg name="play_rate" default="1.0" />
	Play rate for the rosbag.
	
	<arg name="use_sim_time" default="true" />
	Use sim time for ROS.

   
Happy mapping!   
   