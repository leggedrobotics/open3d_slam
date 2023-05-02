include "nested_file2.lua"



options = {
  nested_struct = NESTED_STRUCT2,
  someDoubleParam = 1.6,
}


options.nested_struct.intParam = 6
options.someDoubleParam = 2.3


return options
