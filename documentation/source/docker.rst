
.. _docker_ref:


======
Docker
======


.. note::

   Docker docs are not up do date. Stay tuned for docker image.


TODO still 

We provide a docker image with pre-compiled Open3D binaries.

Pulling the Image from Docker Hub
---------------------------------

The image can be pulled from docker hub using

.. code-block:: bash

   docker pull rslethz/rsl-gpu:open3d_slam

(on PC with GPU) or

.. code-block:: bash

   docker pull rslethz/rsl-cpu:open3d_slam

(on PC with CPU only).

Running the Docker Image
------------------------

For running the docker image we recommend the usage of the run.sh script from this `repository <https://github.com/leggedrobotics/rsl_docker>`__.
After building the image (or pulling it from docker hub), this can be done by typing


.. code-block:: bash

   ./bin/run.sh --type=gpu --tag=open3d

or 

.. code-block:: bash

   ./bin/run.sh --type=cpu --tag=open3d



