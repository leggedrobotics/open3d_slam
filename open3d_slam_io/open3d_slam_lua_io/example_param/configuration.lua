include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)

params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.3
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 0.25
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 30.0
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 0.8
params.mapper_localizer.is_use_map_initialization = true


params.odometry.scan_processing.voxel_size = 0.14
params.odometry.scan_processing.downsampling_ratio = 0.9
params.odometry.scan_processing.scan_cropping.cropping_radius_max = 32.9


params.some_unused_shizzle =1
params.another_unused_shizzle =1


return params
