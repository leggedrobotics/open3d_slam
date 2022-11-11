SAVING_PARAMETERS = {
  save_at_mission_end = true,
  save_map = false,
  save_submaps = false,
}

MOTION_COMPENSATION_PARAMETERS = {
  is_undistort_scan = false,
  is_spinning_clockwise = true,
  scan_duration = 0.1,
  num_poses_vel_estimation = 3,
}

VISUALIZATION_PARAMETERS = {
  assembled_map_voxel_size = 0.3,
  submaps_voxel_size = 0.3,
  visualize_every_n_msec = 300.0,
}

GLOBAL_OPTIMIZATION_PARAMETERS = {
  edge_prune_threshold = 0.2,
  loop_closure_preference = 2.0,
  max_correspondence_distance = 1000.0,
  reference_node = 0,
}

--[[
ICP_PARAMETERS = {
  max_correspondence_dist= 1.0,
  knn= 20,
  max_distance_knn= 3.0,
  max_n_iter= 50,
}

LOOP_CLOSURE_CONSISTENCY_CHECK = {
  max_drift_roll = 30.0 --deg
  max_drift_pitch = 30.0 --deg
  max_drift_yaw = 30.0 --deg
  max_drift_x =: 80.0 --meters
  max_drift_y =: 80.0 --meters
  max_drift_z = 40.0 --meters
}

]]--