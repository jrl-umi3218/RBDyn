/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/parsers/common.h>

namespace YAML
{
class Node;
}

namespace rbd
{

namespace parsers
{

class RBDYN_PARSERS_DLLAPI RBDynFromYAML
{
public:
  RBDynFromYAML(const std::string & input,
                ParserInput input_type = ParserInput::File,
                bool fixed = true,
                const std::vector<std::string> & filtered_links = {},
                bool transform_inertia = true,
                const std::string & base_link = "",
                bool with_virtual_links = true,
                const std::string & spherical_suffix = "_spherical",
                bool remove_filtered_links = true);

  ParserResult & result()
  {
    return res;
  }

  operator std::tuple<rbd::MultiBody &, rbd::MultiBodyConfig &, rbd::MultiBodyGraph &>()
  {
    return std::tie(res.mb, res.mbc, res.mbg);
  }

private:
  ParserResult res;
  bool verbose_;
  bool transform_inertia_;
  bool angles_in_degrees_;
  std::string base_link_;
  size_t link_idx_;
  size_t joint_idx_;
  std::map<std::string, rbd::Joint::Type> joint_types_;
  std::vector<std::string> filtered_links_;
  bool remove_filtered_links_;
  bool with_virtual_links_;
  const std::string & spherical_suffix_;
  std::unordered_map<std::string, Material> materials_;
  std::vector<std::string> fixed_links_;
  std::vector<std::string> removed_links_;

  Eigen::Matrix3d makeInertia(double ixx, double iyy, double izz, double iyz, double ixz, double ixy);

  Eigen::Matrix3d MatrixFromRPY(double r, double p, double y);

  void parseFrame(const YAML::Node & frame, const std::string & name, Eigen::Vector3d & xyz, Eigen::Vector3d & rpy);

  void parseInertia(const YAML::Node & inertia, Eigen::Matrix3d & inertia_mat);

  void parseInertial(const YAML::Node & inertial,
                     const std::string & name,
                     double & mass,
                     Eigen::Vector3d & xyz,
                     Eigen::Vector3d & rpy,
                     Eigen::Matrix3d & inertia);

  bool parseGeometry(const YAML::Node & geometry, Geometry & data);

  void parseVisuals(const YAML::Node & visuals,
                    std::map<std::string, std::vector<Visual>> & data,
                    const std::string & name);

  /** Parse a material tag and cache it */
  void parseMaterial(const YAML::Node & material);

  /** Parse a material tag into the output parameter (if the tag is valid)
   *
   * \param cache If true, lookup existing materials in the cache
   */
  void parseMaterial(const YAML::Node & material, Material & out, bool cache = true);

  void parseLink(const YAML::Node & link);

  bool parseJointType(const YAML::Node & type,
                      const std::string & name,
                      rbd::Joint::Type & joint_type,
                      std::string & type_name,
                      bool force_fixed);

  void parseJointAxis(const YAML::Node & axis, const std::string & name, Eigen::Vector3d & joint_axis);

  void parseJointLimits(const YAML::Node & limits,
                        const std::string & name,
                        const rbd::Joint & joint,
                        bool is_continuous);

  void parseJoint(const YAML::Node & joint);
};

RBDYN_PARSERS_DLLAPI ParserResult from_yaml(const std::string & content,
                                            bool fixed = true,
                                            const std::vector<std::string> & filteredLinksIn = {},
                                            bool transformInertia = true,
                                            const std::string & baseLinkIn = "",
                                            bool withVirtualLinks = true,
                                            const std::string & sphericalSuffix = "_spherical");

RBDYN_PARSERS_DLLAPI ParserResult from_yaml_file(const std::string & file_path,
                                                 bool fixed = true,
                                                 const std::vector<std::string> & filteredLinksIn = {},
                                                 bool transformInertia = true,
                                                 const std::string & baseLinkIn = "",
                                                 bool withVirtualLinks = true,
                                                 const std::string & sphericalSuffix = "_spherical");

RBDYN_PARSERS_DLLAPI ParserResult from_yaml(const std::string & content, const ParserParameters & params);

RBDYN_PARSERS_DLLAPI ParserResult from_yaml_file(const std::string & file_path, const ParserParameters & params);

RBDYN_PARSERS_DLLAPI std::string to_yaml(const ParserResult & res);

} // namespace parsers

} // namespace rbd
