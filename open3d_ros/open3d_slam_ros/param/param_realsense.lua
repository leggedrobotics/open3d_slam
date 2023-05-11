include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)

--ODOMETRY
params.odometry.scan_processing.voxel_size = 0.02
params.odometry.scan_processing.downsampling_ratio = 0.6
params.odometry.scan_processing.scan_cropping.cropping_radius_max = 2.0
params.odometry.scan_processing.scan_cropping.cropping_radius_min = 0.0
params.odometry.scan_matching.icp.max_correspondence_dist = 0.2

--MAPPER_LOCALIZER
params.mapper_localizer.is_merge_scans_into_map = false
params.mapper_localizer.is_build_dense_map = true
params.mapper_localizer.is_use_map_initialization = false
params.mapper_localizer.is_print_timing_information = true
params.mapper_localizer.is_attempt_loop_closures = false
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.03
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 0.6
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 2.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 0.0
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 0.1

--MAP_INITIALIZER
params.map_initializer.pcd_file_path = ""
params.map_initializer.init_pose.x = 0.0
params.map_initializer.init_pose.y = 0.0
params.map_initializer.init_pose.z = 0.0
params.map_initializer.init_pose.roll = 0.0
params.map_initializer.init_pose.pitch = 0.0
params.map_initializer.init_pose.yaw = 0.0

--SUBMAP
params.submap.submap_size = 3.0 --meters


--MAP_BUILDER
params.map_builder.map_voxel_size = 0.03
params.map_builder.scan_cropping.cropping_radius_max = 25.0
params.map_builder.space_carving.carve_space_every_n_scans = 1000000000
params.map_builder.space_carving.truncation_distance = 0.45

--DENSE_MAP_BUILDER
params.dense_map_builder.map_voxel_size = 0.01
params.dense_map_builder.scan_cropping.cropping_radius_max = 2.0
params.dense_map_builder.space_carving.carve_space_every_n_scans = 10
params.dense_map_builder.space_carving.truncation_distance = 0.1

--PLACE_RECOGNITION
params.place_recognition.ransac_min_corresondence_set_size = 40
params.place_recognition.max_icp_correspondence_distance = 0.3
params.place_recognition.min_icp_refinement_fitness = 0.7
params.place_recognition.dump_aligned_place_recognitions_to_file = false 
params.place_recognition.min_submaps_between_loop_closures = 2
params.place_recognition.loop_closure_search_radius = 20.0
params.place_recognition.consistency_check.max_drift_roll = 30.0 --deg
params.place_recognition.consistency_check.max_drift_pitch = 30.0 --deg
params.place_recognition.consistency_check.max_drift_yaw = 30.0 --deg
params.place_recognition.consistency_check.max_drift_x = 80.0 --m
params.place_recognition.consistency_check.max_drift_y = 80.0 --m
params.place_recognition.consistency_check.max_drift_z = 40.0 --m

--SAVING
params.saving.save_map = false


return params