include "default_parameters.lua"


configuration = DEFAULT_PARAMETERS

configuration.odometry.scan_processing.voxel_size = 0.14
configuration.odometry.scan_processing.downsampling_ratio = 0.9
configuration.odometry.scan_processing.scan_cropping.cropping_radius_max = 32.9
--configuration.odometry.scan_processing.scan_cropping.cropper_type = "MaxRadius"



return configuration
