/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/parsers/common.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <boost/variant.hpp>

#include <Eigen/Core>
#include <string>

namespace tinyxml2
{
class XMLElement;
}

namespace rbd
{

RBDYN_PARSERS_DLLAPI std::vector<double> attrToList(const tinyxml2::XMLElement & dom,
                                                    const std::string & attr,
                                                    const std::vector<double> & def = {});

RBDYN_PARSERS_DLLAPI Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom,
                                                  const std::string & attr,
                                                  const Eigen::Vector3d & def = Eigen::Vector3d(0, 0, 0));

RBDYN_PARSERS_DLLAPI Eigen::Matrix3d RPY(const double & r, const double & p, const double & y);

RBDYN_PARSERS_DLLAPI rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type);

RBDYN_PARSERS_DLLAPI sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);
RBDYN_PARSERS_DLLAPI sva::PTransformd originFromTag(const tinyxml2::XMLElement * dom);

RBDYN_PARSERS_DLLAPI std::string parseMultiBodyGraphFromURDF(ParserResult & res,
                                                             const std::string & content,
                                                             const std::vector<std::string> & filteredLinksIn = {},
                                                             bool transformInertia = true,
                                                             const std::string & baseLinkIn = "",
                                                             bool withVirtualLinks = true,
                                                             const std::string & sphericalSuffix = "_spherical");

RBDYN_PARSERS_DLLAPI ParserResult from_urdf(const std::string & content,
                                            bool fixed = true,
                                            const std::vector<std::string> & filteredLinksIn = {},
                                            bool transformInertia = true,
                                            const std::string & baseLinkIn = "",
                                            bool withVirtualLinks = true,
                                            const std::string & sphericalSuffix = "_spherical");

RBDYN_PARSERS_DLLAPI ParserResult from_urdf_file(const std::string & file_path,
                                                 bool fixed = true,
                                                 const std::vector<std::string> & filteredLinksIn = {},
                                                 bool transformInertia = true,
                                                 const std::string & baseLinkIn = "",
                                                 bool withVirtualLinks = true,
                                                 const std::string & sphericalSuffix = "_spherical");

} // namespace rbd
