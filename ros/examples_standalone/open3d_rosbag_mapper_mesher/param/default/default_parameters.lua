include "parameter_structure_definitions.lua"


DEFAULT_PARAMETERS = {
  odometry = deepcopy(ODOMETRY_PARAMETERS),
  submap = deepcopy(SUBMAP_PARAMETERS),
  map_builder = deepcopy(MAP_BUILDER_PARAMETERS),
  dense_map_builder = deepcopy(MAP_BUILDER_PARAMETERS),
  mapper_localizer = deepcopy(MAPPER_LOCALIZER_PARAMETERS),
  saving = deepcopy(SAVING_PARAMETERS),
  visualization = deepcopy(VISUALIZATION_PARAMETERS),
  motion_compensation = deepcopy(MOTION_COMPENSATION_PARAMETERS),
  global_optimization = deepcopy(GLOBAL_OPTIMIZATION_PARAMETERS),
  map_initializer = deepcopy(MAP_INITIALIZER_PARAMETERS),
  place_recognition = deepcopy(PLACE_RECOGNITION_PARAMETERS),
}