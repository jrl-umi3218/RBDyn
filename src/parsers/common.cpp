#include <RBDyn/parsers/common.h>
#include <RBDyn/parsers/urdf.h>
#include <RBDyn/parsers/yaml.h>

namespace rbd
{
ParserResult from_file(const std::string & file_path,
                       bool fixed,
                       const std::vector<std::string> & filtered_links,
                       bool transform_inertia,
                       const std::string & base_link,
                       bool with_virtual_links,
                       const std::string spherical_suffix)
{
  auto extension_pos = file_path.rfind('.');
  auto extension = file_path.substr(extension_pos + 1);
  if(extension == "yaml" or extension == "yml")
  {
    return rbd::from_yaml_file(file_path, fixed, filtered_links, transform_inertia, base_link, with_virtual_links,
                               spherical_suffix);
  }
  else if(extension == "urdf")
  {
    return rbd::from_urdf_file(file_path, fixed, filtered_links, transform_inertia, base_link, with_virtual_links,
                               spherical_suffix);
  }
  else
  {
    throw std::runtime_error("rbd::from_file: Unkown robot model extension '" + extension
                             + "'. Please provide a yaml, yml or urdf file.");
  }
}
} // namespace rbd