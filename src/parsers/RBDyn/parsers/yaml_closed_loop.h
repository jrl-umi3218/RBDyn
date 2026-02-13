#pragma once

#include <RBDyn/parsers/api.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace rbd
{

namespace parsers
{

YAML::Node yaml_parsing(const std::string & file_path);

} // namespace parsers

} // namespace rbd
