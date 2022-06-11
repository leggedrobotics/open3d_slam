============
Installation
============

1. Install the dependencies
2. Compile using catkin build sytem

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

*Contact:* Julian Nubert (nubertj@ethz.ch)

If no open3d installation is present on the system, *open3d_catkin* is built by compiling the open3d libraries within the catkin workspace.
On a modern desktop computer this takes about 5 minutes. If an Open3D installation exists on the system, 
this is used instead and wrapped as a catkin package automatically. The latter can be particularly useful if multiple workspaces 
with this packages are compiled on the computer, or *open3d_catkin* is intended for longer term usage.

EASY WAY: Install Open3d from a PPA
"""""""""""""""""""""""""""""""""""
You can install Open3d from a PPA. The PPA contains Open3d and all dependencies.

.. code-block:: bash

   sudo add-apt-repository ppa:roehling/open3d
   sudo apt update
   sudo apt install libopen3d-dev

You're done, proceed to the compiling section.

EXPERT WAY: Build open3D from source
"""""""""""""""""""""""""""""""""""""

Follow the build from source :ref:`page <build_from_source_ref>`.


Compilation
------------
