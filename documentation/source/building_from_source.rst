.. _build_from_source_ref:

====================
Building from source
====================

Cmake
-----

.. note::

   Cmake version > 3.18 is required to build open3d_catkin!!!!!
   
First we need to install the correct CMake version.

1. Download the latest tar archive from https://cmake.org/download/

2. Do :code:`tar -xf cmake-<version>-rc4.tar.gz` , where :code:`<version>` can for example be 3.23.1.

3. Then install this cmake version by doing:

.. code-block:: bash

   cd cmake-<version>-rc4.tar.gz
   ./configure
   make -j12
   sudo make install
   
Open3D Dependencies
-------------------

Execute either the original installation script from open3d `script <https://github.com/isl-org/Open3D/blob/v0.13.0/util/install_deps_ubuntu.sh>`__, or our fetched version via sudo :code:`./install_Deps.sh`

You're done, you can proceed with the compilation step :ref:`here <compilation_ref>`.


Optional Step
-------------

Follow this step if you wish to build from source and make a local install.

.. mdinclude:: ../../open3d_catkin/install_open3d.md



You're done, you can proceed with the compilation step :ref:`here <compilation_ref>`.

