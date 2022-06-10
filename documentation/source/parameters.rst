Parameters
==========

odometry
--------
Parameters below affect scan to scan lidar odometry.

 

  scan_matching:
    ``icp_objective`` - Which icp objective to use? Default is *PointToPlane*, another option is *PointToPoint*.
    *PointToPlane* usually has faster convergence.
    
    ``max_correspondence_dist`` - SI unit meters. Max distance for k-d tree to consider when searching for correspondences inside ICP (in meters). This means
    that for each point *p* their corresponing poin *q* can be at most *max_correspondence_dist* far away. With large values you will lose
    some accuracy, small values will result in less robust scan matching.
    
    ``knn_normal_estimation`` - Number of nearest neighbors used for normal estimation if *PointToPlane* ICP variant is used. If you use
    *PointToPoint* this parameter is ignored.
    
    ``max_n_iter`` - Maximal number of iterations for the ICP based scan registration inside odometry module.
  
  scan_processing:
    ``voxel_size`` - SI unit meters. Voxel size that is applied to the raw scan before performing scan matching. Operation applied
    after cropping the scan. If voxel size is <= 0.0 then the voxelizing operation is not applied.
      
    ``downsampling_ratio`` - Ratio between 0.0 and 1.0 for downsampling the input pointcloud before performing scan matching. Operation applied
    after voxelizing the scan.
    
    scan_cropping:
      All scan  cropper types operate on raw scans in the range sensor frame. Cropping is the first operation applied in the pre-processing
      pipeline.
      
      ``cropping_radius_max`` - SI units meters. Maximal radius of points to keep, any points further away than *cropping_radius_max* meters will
      be dropped.
      
      ``cropping_radius_min`` - SI units meters. Minimal radius of points to keep, any points closer than *cropping_radius_min* meters will
      be dropped.
      
      ``min_z`` - SI units meters. Minimal height of points to keep, any points with z coordinate less than *min_z* meters will be droopped.
      
      ``max_z`` - SI units meters. Maximal height of points to keep, any points with z coordinate more than *max_z* meters will be droopped.
      
      ``cropper_type`` - Available croppers types. Options are: *Cylinder*, *MaxRadius*, *MinRadius*, *MinMaxRadius*
  
mapping
-------
Parameters listed below affect scan to map refinement, map building, loop closure and pose graph optimization.

  ``is_print_timing_information`` - If true, it prints timing information in terminal.
  
  ``is_build_dense_map`` - If true, *open3d_slam* will build another, dense map in parallel which can be used for visualization.
  
  ``is_attempt_loop_closures`` - If true, *open3d_slam* attempts to correct drift with loop closures.
  
  ``dump_submaps_to_file_before_after_lc`` - If true, the submaps are saved before and after pose graph optimization (after the loop closure).
  Used for debugging.
  
  ``is_refine_odometry_constraints_between_submaps`` - If true, an additional ICP step is run to align submaps 
  when building odometry constraints.
  
  ``min_movement_between_mapping_steps`` - SI unit meters. If translational movement is less than *min_movement_between_mapping_steps* meters, the 
  scan is not merged into the current active submap. 
  
  ``submaps_num_scan_overlap`` - Number of scans that inserted in both adjacent submaps. The overlaping scans are 
  inserted right after the new submap is created.
	
  scan_to_map_refinement:
    Parameter related to scan mathicng.
    
    ``min_refinement_fitness`` - Number between 0 and 1. 0 means that scan has no overlap with the submap (poor match most likely), 1.0 means
    that all points in the scan have a nearest neighbor in the submap (good match most likely).
    
    scan_matching:
      ``icp_objective`` - same as scan matching for odometry.
      
      ``max_correspondence_dist`` - same as scan matching for odometry.
      
      ``knn_normal_estimation`` - same as scan matching for odometry.
      
      ``max_n_iter`` - same as scan matching for odometry.
      
  submaps:
    ``size`` - SI unit meters. Radius of the map. Once the submap is greater than this size,
    we start building a new submap.
    
    ``min_num_range_data`` - minimum number of scans accumulated in a submap.
    
    ``adjacency_based_revisiting_min_fitness`` - when you revisit a submap, we compute the overlap between
    current scan and the submap beng revisited. If it is bigger  than *adjacency_based_revisiting_min_fitness*, then
    the revisited submap becomes a new active submap.

  map_builder:
    Parameters related to scan accumulation (map building) and space carving (pruning). We take the scan
    that was pre proceed in the scan matching step, crop it again and aggregate into the active submap.
    The active submap is then voxelized to keep its size reasonable.
    
    scan_cropping:
      ``cropping_radius_max`` - same as scan matching for odometry.
      
      ``cropping_radius_min`` - same as scan matching for odometry.
      
      ``min_z`` - same as scan matching for odometry.
      
      ``max_z`` - same as scan matching for odometry.
      
      ``cropper_type`` - same as scan matching for odometry.
      
    ``map_voxel_size`` - SI unit meters. Voxel size for all submaps. Note that this is different
    parameter than the voxel size of the scan matcher.
    
    space_carving:
      ``voxel_size`` - SI unit meters. We trace a ray and we keep track what voxels does this map
      hit in space. Every point within hit voxel will be erased. Bigger voxel size result in more aggressive
      pruning.
      
      ``max_raytracing_length`` - SI unit meters. Maximal length to trace a ray from the range sensor.
      
      ``truncation_distance`` - SI unit meters. Stop raytracing once you're *truncation_distance* far from the
      end of the ray.
      
      ``carve_space_every_n_scans`` - Since space carving is computationally expensive, perform it only
      after having merged *carve_space_every_n_scans* in the submap.
      
      ``min_dot_product_with_normal`` - Remove the point only if the dot product of ray (from the origin
      of the range sensor) and surface normal of the ponit we want to remove are big enough. Intuitively,
      if the ray is almost parallel to the surface it would cause many points to be removed (we want to avoid this).
      
  dense_map_builder:
    You can build another map in parallel to the main map. This map can be then very dense, which is sometimes
    nice for visualizatin purposes. For building the dense map, we take the raw scan, crop it and insert it into
    the dense map. No additional pre-processing steps are applied. 
    
    scan_cropping:
      ``cropping_radius_max`` - see map_builder parameters.
      
      ``cropping_radius_min`` - see map_builder parameters.
      
      ``min_z`` - see map_builder parameters.
      
      ``max_z`` - see map_builder parameters.
      
      ``cropper_type`` - see map_builder parameters.
      
    ``map_voxel_size`` - see map_builder parameters.
    
    space_carving:
      ``voxel_size`` - see map_builder parameters.
      
      ``max_raytracing_length`` - see map_builder parameters.
      
      ``truncation_distance`` - see map_builder parameters.
      
      ``carve_space_every_n_scans`` - see map_builder parameters.
      
      ``min_dot_product_with_normal`` - see map_builder parameters.

local_map
---------
  
motion_compensation
-------------------
  
  
visualization
-------------

  
saving_parameters
-----------------



  

    