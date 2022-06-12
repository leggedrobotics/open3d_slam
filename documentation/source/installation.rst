============
Installation
============

1. Install the dependencies
2. Compile using catkin build system

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


.. _compilation_ref:

Compilation
------------

Compilation of *open3d_catkin* is then really straightforward:

.. code-block:: bash

	catkin build open3d_catkin
	
	
The 3 compilation options are chosen automatically in the following order:

1. *open3d* is installed locally and the *$Open3D_DIR* environment variable is pointing to the installation location. 
   The success of this is indicated through the message *INFO: Found manually set path to Open3D. Using version located at (some user defined location)*
   

2. *open3d* is installed globally. Both of these options should compile within a few seconds.


3. If none of the before cases holds, *open3d* is automatically pulled locally and compiled inside the workspace. on an Intel i9 12900K this takes roughly 4min 30s.

Once you have build *open3D_catkin* you can build *open3d_slam* and *open3d_slam_ros* if you want ROS support.

.. code-block:: bash

	catkin build open3d_slam
	catkin build open3d_slam_ros

Usage of open3d_catkin in your project
--------------------------------------

Usage in your catkin project is then straightforward.

**CMakeLists.txt**


.. code-block:: cmake

   set(CATKIN_PACKAGE_DEPENDENCIES
     open3d_catkin
   )
   ...
   find_package(catkin REQUIRED COMPONENTS
     ${CATKIN_PACKAGE_DEPENDENCIES}
   )
   ...
   	
   catkin_package(
     ...
     CATKIN_DEPENDS
       ${CATKIN_PACKAGE_DEPENDENCIES}
     DEPENDS 
   )
   ...
   include_directories(
     ${catkin_INCLUDE_DIRS}
     ...
   )
   ...
   target_link_libraries(${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ...
   )

An example of using ope3d_catkin in other projects can be seen in .
`../../open3d_slam/CMakeLists.txt <https://github.com/leggedrobotics/open3d_slam/blob/master/open3d_slam/CMakeLists.txt>`__.

Code Usage
~~~~~~~~~~

Headers from open3d can then be included as usual:

.. code-block:: cpp

   #include <open3d/...>

