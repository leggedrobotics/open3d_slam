# Open3D SLAM: A Flexible Pointcloud-based SLAM System for Education

open3d_slam is a C++ (cpp) library for SLAM with ROS integration. 

*Contact:* Edo Jelavic (jelavice@ethz.ch)

![title_img](_images/o3d_slam.png)


The main difference between open3d_slam and any other SLAM library out there is that open3d_slam was designed
to be simple and used for education purposes. In fact open3d_slam uses only well established pointcloud processing
algorithms in their basic form which makes it easier for mapping noobs to understand the underlying principles.

You can check out [poster+abstract](https://www.research-collection.ethz.ch/handle/20.500.11850/551852) from ICRA 2022 workshop to
better understand the motivation.

We base our implementation on [Open3D](http://www.open3d.org/), a well maintained and highly performant library for
3D data processing.


The documentation can be found here [open3d_slam Documentation](https://open3d-slam.readthedocs.io/en/latest/).


--------------------------------------

## Docker
We provide a docker image with pre-compiled Open3D binaries.

### Pulling the Image from Docker Hub

The image can be pulled from docker hub using
```bash
docker pull rslethz/rsl-gpu:open3d_slam
```
(on PC with GPU) or
```bash
docker pull rslethz/rsl-cpu:open3d_slam
```
(on PC with CPU).

### Running the Docker Image

For running the docker image we recommend the usage of the run.sh script from this [repository](https://github.com/leggedrobotics/rsl_docker).

After building the image (or pulling it from docker hub), this can be done by typing
```bash
./bin/run.sh --type=gpu --tag=open3d
```
or
```bash
./bin/run.sh --type=cpu --tag=open3d
```
