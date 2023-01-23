include "nested_file.lua"

EXAMPLE_STRUCT = {
  boolParam = true,
  intParam = 4,
  structParam = NESTED_STRUCT,
}

options = {
  example_struct = EXAMPLE_STRUCT,
  some_double_shizzle = 1.6,
}

return options
