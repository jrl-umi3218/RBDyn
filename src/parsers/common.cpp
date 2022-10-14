#include <RBDyn/parsers/common.h>
#include <RBDyn/parsers/urdf.h>
#include <RBDyn/parsers/yaml.h>

namespace rbd
{

namespace parsers
{

ParserResult from_file(const std::string & file_path,
                       bool fixed,
                       const std::vector<std::string> & filtered_links,
                       bool transform_inertia,
                       const std::string & base_link,
                       bool with_virtual_links,
                       const std::string spherical_suffix)
{
  return from_file(file_path, ParserParameters{}
                                  .fixed(fixed)
                                  .filtered_links(filtered_links)
                                  .transform_inertia(transform_inertia)
                                  .base_link(base_link)
                                  .remove_virtual_links(!with_virtual_links)
                                  .spherical_suffix(spherical_suffix));
}

ParserResult from_file(const std::string & file_path, const ParserParameters & params)
{
  auto extension_pos = file_path.rfind('.');
  auto extension = file_path.substr(extension_pos + 1);
  if(extension == "yaml" || extension == "yml")
  {
    return from_yaml_file(file_path, params);
  }
  else if(extension == "urdf")
  {
    return from_urdf_file(file_path, params);
  }
  else
  {
    throw std::runtime_error("rbd::parsers::from_file: Unkown robot model extension '" + extension
                             + "'. Please provide a yaml, yml or urdf file.");
  }
}

std::string prefix_path(const std::string & path)
{
  if(path.rfind("package://", 0) == 0 || path.rfind("file://", 0) == 0)
  {
    return path;
  }
  else
  {
    return std::string{"file://"} + path;
  }
}

} // namespace parsers

} // namespace rbd
