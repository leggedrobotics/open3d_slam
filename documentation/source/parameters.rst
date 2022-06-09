Parameters
=====

odometry
----
Parameters below affect scan to scan lidar odometry.

 

  scan_matching:
    ``icp_objective`` - Which icp objective to use? Default is *PointToPlane*, another option is *PointToPoint*.
    *PointToPlane* usually has faster convergence.
    
    ``max_correspondence_dist``
    
    ``knn_normal_estimation``
    
    ``max_n_iter``
  
  scan_processing:
    ``voxel_size``
      
    ``downsampling_ratio``
    
    scan_cropping:
      ``cropping_radius_max``
      
      ``cropping_radius_min``
      
      ``min_z``
      
      ``max_z``
      
      ``cropper_type``
  
mapping
----


local_map
----
  
motion_compensation
----
  
  
visualization
----

  
saving_parameters
----



  

    