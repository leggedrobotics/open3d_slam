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

2. GLFW
~~~~~~~

.. code-block:: bash

   sudo apt-get install libglfw3 libglfw3-dev

3. Ros dependencies
~~~~~~~~~~~~~~~~~~~
If you want to use open3d_slam_ros consider installing jsk-rviz plugins. We use them to visualize the trajectories. 
If you don't have them installed, you should still be able to run the package, however with some red letters in the terminal.

Install with apt:

.. code-block:: bash

   sudo apt install ros-noetic-jsk-rviz-plugins

In case this installation is not sufficient and you still get errors, follow the instructions:

.. code-block:: bash

   wget http://archive.ubuntu.com/ubuntu/pool/universe/p/pcl/libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb
   wget http://de.archive.ubuntu.com/ubuntu/pool/universe/v/vtk7/libvtk7.1p_7.1.1+dfsg2-2ubuntu1_amd64.deb
   sudo dpkg -i libvtk7.1p_7.1.1+dfsg2-2ubuntu1_amd64.deb
   sudo dpkg -i libpcl-visualization1.10_1.10.0+dfsg-5ubuntu1_amd64.deb

jsk-rviz plugins inherently depends on pcl_visualization package which depends on vtk7 package. However, these packages are not part of the PCL debian packages. Therefore, we need to install them manually.

4. LUA
~~~~~~~~~~~~~~~~~~~
If you use open3d_slam_ros, the configs are loaded using LUA, as they allow for an easy configuration management with minimal duplicated code.

Install with apt:

.. code-block:: bash

   sudo apt install liblua5.2-dev

5. open3d_catkin
~~~~~~~~~~~~~~~~

Follow the instructions to build :ref:`open3d_catkin <open3d_catkin_ref>`.


.. _compilation_ref:

Compilation
------------


Once you have built *open3D_catkin* you can build *open3d_slam* and *open3d_slam_ros* if you want ROS support.

.. code-block:: bash

	catkin build open3d_slam -DCMAKE_BUILD_TYPE=Release
	catkin build open3d_slam_ros -DCMAKE_BUILD_TYPE=Release


**You're done with open3d_slam installation**, you can proceed to the usage site.
