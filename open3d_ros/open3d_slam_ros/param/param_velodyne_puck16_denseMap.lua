include "param_velodyne_puck16.lua"

params = deepcopy(params)

--MAPPER_LOCALIZER
params.mapper_localizer.is_build_dense_map = true

--MAP_BUILDER
params.map_builder.space_carving.carve_space_every_n_scans = 10

--DENSE_MAP_BUILDER
params.dense_map_builder.map_voxel_size = 0.05
params.dense_map_builder.scan_cropping.cropping_radius_max = 15.0
params.dense_map_builder.space_carving.carve_space_every_n_scans = 10
params.dense_map_builder.space_carving.truncation_distance = 0.4
params.dense_map_builder.space_carving.voxel_size = params.dense_map_builder.map_voxel_size
--params.dense_map_builder.space_carving.max_raytracing_length = 10.0

return params