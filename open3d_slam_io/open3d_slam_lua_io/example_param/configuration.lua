include "default_parameters.lua"


o3d_params = DEFAULT_PARAMETERS

o3d_params.mapper_localizer.is_use_map_initialization = true
o3d_params.odometry.scan_processing.voxel_size = 0.14
o3d_params.odometry.scan_processing.downsampling_ratio = 0.9
o3d_params.odometry.scan_processing.scan_cropping.cropping_radius_max = 32.9
--o3d_params.odometry.scan_processing.scan_cropping.cropper_type = "MaxRadius"
o3d_params.some_unused_shizzle =1


return o3d_params
