# Open3D SLAM: A Flexible Pointcloud-based SLAM System for Education

This package contains the code for a open3D SLAM framework.

*Contact:* Edo Jelavic (jelavice@ethz.ch)

![title_img](documentation/images/o3d_slam.png)

Documentation (work in progress):
The documentation can be found here [open3d_slam Documentation](https://open3d-slam.readthedocs.io/en/latest/).

DISCLAIMER  
This package is still under heavy development. Hence we cannot (yet) guarantee any stability.

## Dependencies

### open3d_catkin
For compling _open3d_catkin_ please follow the instructions in [open3d_catkin/](open3d_catkin/).

### Glog
```bash
sudo apt install libgoogle-glog-dev
```

### Ros dependencies
If you want to use _open3d_slam_ros_ consider installing jsk-rviz plugins. We use them to visualize the trajectories. If you don't have them installed, you should still be able to run the package, however with some red letters in the temrinal.
```bash
sudo apt install ros-noetic-jsk-rviz-plugins
```

## Compilation
### Open3d SLAM
Compiling the _open3d_slam_ libraries can be done via
```bash
catkin build open3d_slam
```

### Open3d SLAM ROS
If open3d_slam should be used with ROS, the provided ROS wrapper can be used.
It can be compiled via
```bash
catkin build open3d_slam_ros
```

## Running the Code

Add the required config to the launch file `mapping.launch` or pass them as an argument.
The main parameters to adapt are
1. _parameter_file_path_: This file contains the hyperparameters of the deployed sensor during the registration.
2. _cloud_topic_: This is the name of the pc2-pointcloud topic of the LiDAR sensor.

After sourcing the workspace the code can then be run using
```bash
roslaunch open3d_slam mapping.launch
```

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
