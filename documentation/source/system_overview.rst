===============
System Overview
===============


Open3D SLAM is a pointcloud based SLAM system. It takes pointclouds from various sensor modalities (e.g. LiDAR or depth camera) and produces a 
globally consistent map of the environment. An overview of the system is given in the image below:


.. image:: ../images/o3d_slam_overview.png
   :scale: 45 %
   :alt: Schematic of open3d_slam system
   :align: center

The scans are forwarded to the odometry module that performs scan2scan matching to estimate ego motion. The odometry is used as an initial guess for
the scan to map refinement which estimates ego motion as well as builds the map of the environment. The map is divided into
submaps and *open3d_slam* builds pose graph by introducing constraints between different submaps.


Scan Matching
-------------

Done using ICP for both scan2scan and scan2map. For documentations and tutorials on ICP you can refer to the original open3D 
`documentation <http://www.open3d.org/docs/latest/tutorial/Basic/icp_registration.html>`__.

Place Recognition
-----------------

We rely on `RANSAC <http://www.open3d.org/docs/latest/tutorial/Advanced/global_registration.html>`__ and `FPFH <https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html>`__ features.
As soon as the submap is finished, we match it against other finished submaps in proximity (low drift assumption). The place recognition module runs
RANSAC feature matching and looks at number of inliers (too low and we reject the loop closure). In addition, *open3d_slam* refines the
RANSAC registration with ICP (if fitness score is too low, the loop closure is rejected).

Pose Graph Optimization
-----------------------

We use a pose graph backend from open3D. You can check the original documentation `here <http://www.open3d.org/docs/latest/tutorial/Advanced/multiway_registration.html>`__ .
