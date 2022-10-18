# Open3D SLAM: A Flexible Pointcloud-based SLAM System for Education

open3d_slam is a C++ (cpp) library for SLAM with ROS integration. 

**Main Contact:** Edo Jelavic ([jelavice@ethz.ch](mailto:jelavice@ethz.ch?subject=[GitHub]))

**Authors:** [Edo Jelavic](https://rsl.ethz.ch/utils/search.MjAyNjMy.html), [Julian Nubert](https://juliannubert.com/), [Marco Hutter](https://rsl.ethz.ch/the-lab/people/person-detail.MTIxOTEx.TGlzdC8yNDQxLC0xNDI1MTk1NzM1.html)

**Poster and Abstract:** [link](https://www.research-collection.ethz.ch/handle/20.500.11850/551852)

**Documentation:** [link](https://open3d-slam.readthedocs.io/en/latest/)

![title_img](documentation/images/o3d_slam.png)


The main difference between open3d_slam and other SLAM libraries out there is that open3d_slam was designed
to be simple and used for education purposes. In fact, open3d_slam uses only well-established algorithms in their basic form.
We hope that this will make it easier for newcomers to enter the field. It works with pointclouds, no additional input such as IMU is required. Open3D_slam can build a map from scratch or localize in a given map. The given map can also be extended with new measurements.

We base our implementation on [Open3D](http://www.open3d.org/), a well-maintained and highly performant library for
3D data processing.

The documentation and example datasets can be found here [open3d_slam Documentation](https://open3d-slam.readthedocs.io/en/latest/).

We provide a catkin wrapper for Open3D such that you can easily use Open3D in your ROS projects. See documentation in
[open3d_catkin/README.md](https://github.com/leggedrobotics/open3d_slam/tree/master/open3d_catkin).

If you find this work useful, or use it for your research, please consider citing the corresponding work:
```
@inproceedings{jelavic2022open3d,
  title={Open3D SLAM: Point Cloud Based Mapping and Localization for Education},
  author={Jelavic, Edo and Nubert, Julian and Hutter, Marco},
  booktitle={Robotic Perception and Mapping: Emerging Techniques, ICRA 2022 Workshop},
  pages={24},
  year={2022},
  organization={ETH Zurich, Robotic Systems Lab}
}
```
