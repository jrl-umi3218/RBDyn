#include <RBDyn/parsers/yaml_closed_loop.h>

namespace rbd
{

namespace parsers
{

YAML::Node yaml_parsing(const std::string & file_path)
{
  YAML::Node config;
  if(std::ifstream(file_path))
  {
    try
    {
      config = YAML::LoadFile(file_path);
      return config;
    }
    catch(const std::exception & e)
    {
      std::cerr << "Failed to parse YAML file " << file_path << ": " << e.what() << std::endl;
    }
  }
  return config;
}

} // namespace parsers

} // namespace rbd
