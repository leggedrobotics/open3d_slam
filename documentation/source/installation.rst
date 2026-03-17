============
Installation
============

The repository targets ROS 2 Jazzy only.

You can either build directly in a local workspace or use the provided Docker image described in :ref:`docker <docker_ref>`.

Workspace setup
---------------

1. Install the build tools:

.. code-block:: bash

   sudo apt install python3-colcon-common-extensions python3-rosdep

2. Create a workspace and clone the repository into `src`:

.. code-block:: bash

   mkdir -p ~/open3d_slam_ws/src
   cd ~/open3d_slam_ws/src
   git clone https://github.com/leggedrobotics/open3d_slam.git
   cd ..

3. Source ROS 2 Jazzy:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
4. Build the Jazzy workspace:

Follow the instructions to build :ref:`open3d_colcon <open3d_colcon_ref>`.


.. _compilation_ref:

Compilation
------------


Once Open3D is configured, build the Jazzy packages with `colcon`:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-up-to open3d_slam_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

**You're done with open3d_slam installation**, you can proceed to the usage site.
