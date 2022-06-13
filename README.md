# Open3D SLAM: A Flexible Pointcloud-based SLAM System for Education

open3d_slam is a C++ (cpp) library for SLAM with ROS integration. 

*Contact:* Edo Jelavic (jelavice@ethz.ch)

![title_img](_images/o3d_slam.png)


The main difference between open3d_slam and other SLAM libraries out there is that open3d_slam was designed
to be simple and used for education purposes. In fact open3d_slam uses only well established algorithms in their basic form.
We hope that this will make it easier for newcomers to enter the filed.

You can check out [poster+abstract](https://www.research-collection.ethz.ch/handle/20.500.11850/551852) from ICRA 2022 workshop to
better understand the motivation.

We base our implementation on [Open3D](http://www.open3d.org/), a well maintained and highly performant library for
3D data processing.

The documentation can be found here [open3d_slam Documentation](https://open3d-slam.readthedocs.io/en/latest/).

We provide a catkin wrapper for Open3D such that you can easily use Open3D in your ROS projects. See documentation in
[open3d_catkin/README.md](https://github.com/leggedrobotics/open3d_slam/tree/master/open3d_catkin).

