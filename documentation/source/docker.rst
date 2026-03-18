
.. _docker_ref:


======
Docker
======

This repository now ships a single Jazzy development image in ``open3d_slam.dockerfile``.
The image builds Open3D 0.15.1 from source and compiles the full Jazzy workspace during ``docker build``.

Building the Image
------------------

.. code-block:: bash

   docker build -f open3d_slam.dockerfile -t open3d_slam:jazzy .

Running the Docker Image
------------------------

The image drops you into a shell with the repository checked out under ``/opt/open3d_slam_ws/src/open3d_slam``.

.. code-block:: bash

   docker run --rm -it open3d_slam:jazzy
