[1mdiff --git a/open3d_slam_ros/include/open3d_slam_ros/SlamMapInitializer.hpp b/open3d_slam_ros/include/open3d_slam_ros/SlamMapInitializer.hpp[m
[1mindex 6a6256b..5dde60a 100644[m
[1m--- a/open3d_slam_ros/include/open3d_slam_ros/SlamMapInitializer.hpp[m
[1m+++ b/open3d_slam_ros/include/open3d_slam_ros/SlamMapInitializer.hpp[m
[36m@@ -34,8 +34,8 @@[m [mprivate:[m
 	void initInterectiveMarker();[m
 	void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);[m
 	visualization_msgs::InteractiveMarker createInteractiveMarker() const;[m
[31m-	[m
[31m-  interactive_markers::MenuHandler menuHandler_;[m
[32m+[m[41m  [m
[32m+[m	[32minteractive_markers::MenuHandler menuHandler_;[m
   interactive_markers::InteractiveMarkerServer server_;[m
 	std::shared_ptr<SlamWrapper> slamPtr_;[m
 	ros::Subscriber cloudSubscriber_;[m
[1mdiff --git a/open3d_slam_ros/param/params_velodyne_puck16.yaml b/open3d_slam_ros/param/params_velodyne_puck16.yaml[m
[1mindex 3b074da..71dca48 100644[m
[1m--- a/open3d_slam_ros/param/params_velodyne_puck16.yaml[m
[1m+++ b/open3d_slam_ros/param/params_velodyne_puck16.yaml[m
[36m@@ -45,9 +45,6 @@[m [mmapping:[m
     initialize_map: true[m
     max_translation_error: 0.5[m
     max_angle_error: 0.314[m
[31m-    # map_frame_id: velodyne[m
[31m-    # map_mesh_path: /rosp_ws/src/rosp/rosp_control/config/corner.dae[m
[31m-    # map_pointcloud_path: /rosp_ws/src/rosp/rosp_control/config/corner.ply[m
 [m
   submaps:[m
     size: 20[m
[1mdiff --git a/open3d_slam_ros/src/SlamMapInitializer.cpp b/open3d_slam_ros/src/SlamMapInitializer.cpp[m
[1mindex 9a2e502..834335f 100644[m
[1m--- a/open3d_slam_ros/src/SlamMapInitializer.cpp[m
[1m+++ b/open3d_slam_ros/src/SlamMapInitializer.cpp[m
[36m@@ -30,11 +30,9 @@[m [mSlamMapInitializer::SlamMapInitializer(std::shared_ptr<SlamWrapper> slamPtr, ros[m
 void SlamMapInitializer::initialize() {[m
   PointCloud raw_map;[m
 [m
[31m-  std::cerr << "intializing" << std::endl;[m
[31m-  frameId_ = nh_->param<std::string>("mapping/initial_map/map_frame_id", "");[m
[31m-	meshResourcePath_ = nh_->param<std::string>("mapping/initial_map/map_mesh_path", "");[m
[31m-  pcdFilePath_ = nh_->param<std::string>("mapping/initial_map/map_pointcloud_path", "");[m
[31m-  std::cerr << "intializing" << std::endl;[m
[32m+[m[32m  frameId_ = nh_->param<std::string>("initial_map/map_frame_id", "");[m
[32m+[m	[32mmeshResourcePath_ = nh_->param<std::string>("initial_map/map_mesh_path", "");[m
[32m+[m[32m  pcdFilePath_ = nh_->param<std::string>("initial_map/map_pointcloud_path", "");[m
 [m
 [m
   initialized_.store(false);[m
[36m@@ -45,10 +43,6 @@[m [mvoid SlamMapInitializer::initialize() {[m
 	{[m
 		std::cerr << "[Error] Initialization pointcloud not loaded" << std::endl;[m
   }[m
[31m-[m
[31m-  std::cerr << "Empty mesh? " << raw_map.IsEmpty() << std::endl;[m
[31m-  std::cerr << "Has points? " << raw_map.HasPoints() << std::endl;[m
[31m-[m
   [m
   while (!initialized_.load())[m
   {[m
[36m@@ -58,7 +52,7 @@[m [mvoid SlamMapInitializer::initialize() {[m
   [m
 }[m
 [m
[31m-void SlamMapInitializer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {[m
[32m+[m[32mvoid SlamMapInitializer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {[m
 	open3d::geometry::PointCloud cloud;[m
 	open3d_conversions::rosToOpen3d(msg, cloud, false);[m
   const Time timestamp = fromRos(msg->header.stamp);[m
