include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)

-- ODOMETRY
params.odometry.scan_processing.voxel_size = 0.10
params.odometry.scan_processing.downsampling_ratio = 0.45
params.odometry.scan_processing.scan_cropping.cropping_radius_min = 1.5
params.odometry.scan_processing.scan_cropping.cropping_radius_max = 28.0
params.odometry.scan_matching.icp.max_correspondence_dist = 0.6

-- MAPPER_LOCALIZER
params.mapper_localizer.is_merge_scans_into_map = false
params.mapper_localizer.is_build_dense_map = false
params.mapper_localizer.is_use_map_initialization = false
params.mapper_localizer.is_print_timing_information = true
params.mapper_localizer.scan_to_map_registration.min_refinement_fitness = 0.60
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.15
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 0.55
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 1.5
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 24.0
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 0.7

-- SUBMAP
params.submap.submap_size = 12.0
params.submap.min_num_range_data = 8

-- MAP_BUILDER
params.map_builder.map_voxel_size = 0.12
params.map_builder.scan_cropping.cropping_radius_min = 1.5
params.map_builder.scan_cropping.cropping_radius_max = 22.0
params.map_builder.space_carving.carve_space_every_n_scans = 8
params.map_builder.space_carving.truncation_distance = 0.3

-- DENSE_MAP_BUILDER
params.dense_map_builder.map_voxel_size = 0.08
params.dense_map_builder.scan_cropping.cropping_radius_min = 1.5
params.dense_map_builder.scan_cropping.cropping_radius_max = 18.0
params.dense_map_builder.space_carving.carve_space_every_n_scans = 8
params.dense_map_builder.space_carving.truncation_distance = 0.2

-- PLACE_RECOGNITION
params.place_recognition.ransac_min_corresondence_set_size = 30
params.place_recognition.max_icp_correspondence_distance = 0.25
params.place_recognition.min_icp_refinement_fitness = 0.65
params.place_recognition.loop_closure_search_radius = 12.0
params.place_recognition.min_submaps_between_loop_closures = 3

-- VISUALIZATION
params.visualization.assembled_map_voxel_size = 0.2
params.visualization.submaps_voxel_size = 0.2
params.visualization.visualize_every_n_msec = 500.0

-- SAVING
params.saving.save_at_mission_end = false
params.saving.save_map = false
params.saving.save_submaps = false


return params
