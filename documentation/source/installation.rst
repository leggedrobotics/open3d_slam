============
Installation
============

1. Install the dependencies
2. Compile using catkin build system

or 

run in a docker. In that case, check the instructions for :ref:`docker <docker_ref>`.

Dependencies
------------

1. Glog
~~~~~~~

.. code-block:: bash

   sudo apt install libgoogle-glog-dev

2. Ros dependencies
~~~~~~~~~~~~~~~~~~~
If you want to use open3d_slam_ros consider installing jsk-rviz plugins. We use them to visualize the trajectories. 
If you don't have them installed, you should still be able to run the package, however with some red letters in the temrinal.

Install with apt:

.. code-block:: bash

   sudo apt install ros-noetic-jsk-rviz-plugins

3. open3d_catkin
~~~~~~~~~~~~~~~~

Follow the instructinos to build :ref:`open3d_catkin <open3d_catkin_ref>`.


.. _compilation_ref:

Compilation
------------


Once you have build *open3D_catkin* you can build *open3d_slam* and *open3d_slam_ros* if you want ROS support.

.. code-block:: bash

	catkin build open3d_slam
	catkin build open3d_slam_ros


**You're done with open3d_slam installation**, you can proceed to the usage site.
