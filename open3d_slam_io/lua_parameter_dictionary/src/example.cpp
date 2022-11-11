/*
 * example.cpp
 *
 *  Created on: Mar 26, 2021
 *      Author: jelavice
 */

#include <ros/package.h>
#include <iostream>
#include <memory>
#include "lua_parameter_dictionary/configuration_file_resolver.h"
#include "lua_parameter_dictionary/lua_parameter_dictionary.h"

int main(int argc, char** argv) {
  using namespace lua_dict;
  const std::string path = ros::package::getPath("lua_parameter_dictionary") + "/config/";
  const std::vector<std::string> paths({path});
  auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);

  const std::string fullPath = fileResolver->GetFullPathOrDie("example.lua");
  const std::string fullContent = fileResolver->GetFileContentOrDie("example.lua");

  std::cout << "Loaded full path: " << fullPath << std::endl;
  std::cout << "Loaded content: " << fullContent << "\n \n";

  const std::string code = fullContent;
  LuaParameterDictionary dict(fullContent, std::move(fileResolver));

  std::cout << "keys in the dictionary:  \n";
  for (auto key : dict.GetKeys()) {
    std::cout << key << ", ";
  }
  std::cout << "\n";
  std::cout << "some_double_shizzle=" << dict.GetDouble("some_double_shizzle") << "\n \n";
  std::unique_ptr<LuaParameterDictionary> sub_dict(dict.GetDictionary("example_struct"));
  std::cout << "keys in the sub dictionary: \n";
  for (auto key : sub_dict->GetKeys()) {
    std::cout << key << ", ";
  }
  std::cout << "\n";
  std::cout << "boolParam=" << std::boolalpha << sub_dict->GetBool("boolParam") << "\n";
  std::cout << "intParam=" << sub_dict->GetInt("intParam") << "\n \n";

  auto sub_sub_dict(sub_dict->GetDictionary("structParam"));
  std::cout << "keys in the sub sub dictionary: \n";
  for (auto key : sub_sub_dict->GetKeys()) {
    std::cout << key << ", ";
  }
  std::cout << "\n";
  std::cout << "boolParam=" << std::boolalpha << sub_sub_dict->GetBool("boolParam") << "\n";
  std::cout << "intParam=" << sub_sub_dict->GetInt("intParam") << "\n \n";

  auto sub_sub_sub_dict(sub_sub_dict->GetDictionary("nestedNestedStruct"));
  std::cout << "keys in the sub sub sub dictionary: \n";
  for (auto key : sub_sub_sub_dict->GetKeys()) {
    std::cout << key << ", ";
  }
  std::cout << "\n";
  std::cout << "doubleParam=" << sub_sub_sub_dict->GetDouble("doubleParam") << "\n";
  std::cout << "intParam=" << sub_sub_sub_dict->GetInt("intParam") << "\n \n";

  {
    auto fileResolver = std::make_unique<ConfigurationFileResolver>(paths);
    const std::string basename = "example_override.lua";
    const std::string fullContent = fileResolver->GetFileContentOrDie(basename);
    const std::string fullPath = fileResolver->GetFullPathOrDie(basename);
    const std::string code = fullContent;
    LuaParameterDictionary dict(fullContent, std::move(fileResolver));
    std::cout << " int param: " << dict.GetDictionary("nested_struct")->GetInt("intParam") << "\n";
    std::cout << "double param: " << dict.GetDouble("someDoubleParam") << std::endl;
  }
  std::cout << "All done" << std::endl;
  return 0;
}
