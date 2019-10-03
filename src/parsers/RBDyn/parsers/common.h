/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <rbdyn/config.hh>

#include <SpaceVecAlg/SpaceVecAlg>

#include <boost/variant.hpp>

#include <Eigen/Core>
#include <string>

namespace rbd
{

enum class RBDYN_DLLAPI ParserInput
{
  File,
  Description
};

struct RBDYN_DLLAPI Limits
{
public:
  std::map<std::string, std::vector<double>> lower;
  std::map<std::string, std::vector<double>> upper;
  std::map<std::string, std::vector<double>> velocity;
  std::map<std::string, std::vector<double>> torque;
};

struct RBDYN_DLLAPI Geometry
{
public:
  struct Mesh
  {
    Mesh() : scale(1) {}
    std::string filename;
    double scale;
  };
  struct Box
  {
    Box() : size(Eigen::Vector3d::Zero()) {}
    Eigen::Vector3d size;
  };
  struct Cylinder
  {
    Cylinder() : radius(0.), length(0.) {}
    double radius;
    double length;
  };
  struct Sphere
  {
    Sphere() : radius(0.) {}
    double radius;
  };

  //! \brief A Superellipsoid can approximate primitive shapes as boxes, cylinders and pyrimids while remaining strictly
  //! convex.
  //!
  //! An illustration of superellipsoids can be found here
  //! https://en.wikipedia.org/wiki/Superellipsoid#/media/File:Superellipsoid_collection.png, with epsilon1 = e/2 and
  //! epsilon2 = n/2. The size of the superellipsoid is half its bounding box
  //!
  //! Superellipsoids are not part of the URDF standard and so won't probably be available in other URDF parsers
  struct Superellipsoid
  {
    Superellipsoid() : size(Eigen::Vector3d::Zero()), epsilon1(1), epsilon2(1) {}
    Eigen::Vector3d size;
    double epsilon1, epsilon2;
  };

  enum Type
  {
    BOX,
    CYLINDER,
    SPHERE,
    MESH,
    SUPERELLIPSOID,
    UNKNOWN
  };
  Type type;
  boost::variant<Box, Cylinder, Mesh, Sphere, Superellipsoid> data;

  Geometry() : type(UNKNOWN) {}
};

struct RBDYN_DLLAPI Visual
{
  std::string name;
  sva::PTransformd origin;
  Geometry geometry;
};

struct RBDYN_DLLAPI ParserResult
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  rbd::Limits limits;
  std::map<std::string, std::vector<rbd::Visual>> visual;
  std::map<std::string, std::vector<rbd::Visual>> collision;
  std::map<std::string, sva::PTransformd> collision_tf;
  std::string name;
};

//! \brief Checks the file extension and parses it as URDF or YAML accordingly
//!
//! \param file_path Path to the file to parse
//! \return ParserResult The parsing result
ParserResult from_file(const std::string & file_path,
                       bool fixed = true,
                       const std::vector<std::string> & filtered_links = {},
                       bool transform_inertia = true,
                       const std::string & base_link = "",
                       bool with_virtual_links = true,
                       const std::string spherical_suffix = "_spherical");

} // namespace rbd
