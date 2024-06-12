/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/parsers/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <boost/variant.hpp>

#include <Eigen/Core>
#include <string>

namespace rbd
{

namespace parsers
{

enum class ParserInput
{
  File,
  Description
};

struct RBDYN_PARSERS_DLLAPI Limits
{
public:
  std::map<std::string, std::vector<double>> lower;
  std::map<std::string, std::vector<double>> upper;
  std::map<std::string, std::vector<double>> velocity;
  std::map<std::string, std::vector<double>> torque;
};

struct RBDYN_PARSERS_DLLAPI Geometry
{
public:
  struct Mesh
  {
    Mesh() : scaleV(Eigen::Vector3d::Ones()) {}
    std::string filename;
    Eigen::Vector3d scaleV;
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
  using Data = boost::variant<Box, Cylinder, Mesh, Sphere, Superellipsoid>;
  Data data;

  Geometry() : type(UNKNOWN) {}
};

/** A material, as specified in the URDF format, is optionally attached to a visual element
 *
 * It can be either:
 * - NONE if there is not material attached to the visual element
 * - COLOR if it is a color element (rgba values in the [0, 1] range)
 * - TEXTURE if it is a texture (filename)
 */
struct RBDYN_PARSERS_DLLAPI Material
{
  struct Color
  {
    double r;
    double g;
    double b;
    double a;
  };
  struct Texture
  {
    std::string filename;
  };

  enum class Type
  {
    NONE,
    COLOR,
    TEXTURE
  };
  Type type;
  using Data = boost::variant<Color, Texture>;
  Data data;

  Material() : type(Type::NONE) {}
};

struct RBDYN_PARSERS_DLLAPI Visual
{
  std::string name;
  sva::PTransformd origin;
  Geometry geometry;
  Material material;
};

/** Options passed to the parsing function */
struct RBDYN_PARSERS_DLLAPI ParserParameters
{
  /** Create a root free joint if false, fixed otherwise */
  bool fixed_ = true;

  /** The bodies in this list will be filtered, the exact behavior depends on \ref remove_filtered_links
   *
   * If true (default): the links in this list do not appear in the resulting MultiBody
   *
   * If false: the links in this list appear in the resulting MultiBody but the joints they are attached to are fixed
   */
  std::vector<std::string> filtered_links_ = {};

  /** Control the link filter behavior
   *
   * If true (default): the links in \ref filtered_links_ do not appear in the resulting MultiBody
   *
   * If false: the links in \ref filtered_links_ appear in the resulting MultiBody but the joints they are attached to
   * are fixed
   */
  bool remove_filtered_links_ = true;

  /** If true, the inertia of links are moved to their CoM frame */
  bool transform_inertia_ = true;

  /** If non-empty, use this body as the base for the MultiBody, otherwise the first body in the URDF/YAML is used */
  std::string base_link_ = "";

  /** If true, bodies without inertial parameters are removed from the resulting MultiBody
   *
   * \note This is independent of the \ref remove_filtered_links parameter
   */
  bool remove_virtual_links_ = false;

  /** Treat joint with this suffix as spherical joints */
  std::string spherical_suffix_ = "_spherical";

  /** Change the \ref fixed_ parameter to \param fixed and returns self */
  inline ParserParameters & fixed(bool fixed) noexcept
  {
    fixed_ = fixed;
    return *this;
  }

  /** Change the \ref filtered_links_ parameter to \param links and returns self */
  inline ParserParameters & filtered_links(const std::vector<std::string> & links) noexcept
  {
    filtered_links_ = links;
    return *this;
  }

  /** Change the \ref remove_filtered_links_ parameter to \param value and returns self */
  inline ParserParameters & remove_filtered_links(bool value) noexcept
  {
    remove_filtered_links_ = value;
    return *this;
  }

  /** Change the \ref transform_inertia_ parameter to \param value and returns self */
  inline ParserParameters & transform_inertia(bool value) noexcept
  {
    transform_inertia_ = value;
    return *this;
  }

  /** Change the \ref base_link_ parameter to \param link and returns self */
  inline ParserParameters & base_link(const std::string & link) noexcept
  {
    base_link_ = link;
    return *this;
  }

  /** Change the \ref remove_virtual_links_ parameter to \param value and returns self */
  inline ParserParameters & remove_virtual_links(bool value) noexcept
  {
    remove_virtual_links_ = value;
    return *this;
  }

  /** Change the \ref spherical_suffix_ parameter to \param suffix and returns self */
  inline ParserParameters & spherical_suffix(const std::string & suffix) noexcept
  {
    spherical_suffix_ = suffix;
    return *this;
  }
};

struct RBDYN_PARSERS_DLLAPI ParserResult
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  Limits limits;
  std::map<std::string, std::vector<Visual>> visual;
  std::map<std::string, std::vector<Visual>> collision;
  std::string name;
};

//! \brief Checks the file extension and parses it as URDF or YAML accordingly
//!
//! \param file_path Path to the file to parse
//! \return ParserResult The parsing result
RBDYN_PARSERS_DLLAPI ParserResult from_file(const std::string & file_path,
                                            bool fixed = true,
                                            const std::vector<std::string> & filtered_links = {},
                                            bool transform_inertia = true,
                                            const std::string & base_link = "",
                                            bool with_virtual_links = true,
                                            const std::string spherical_suffix = "_spherical");

//! \brief Checks the file extension and parses it as URDF or YAML accordingly
//!
//! \param file_path Path to the file to parse
//! \param params Parser parameters
//! \return ParserResult The parsing result
RBDYN_PARSERS_DLLAPI ParserResult from_file(const std::string & file_path, const ParserParameters & params);

/**
 * \brief Ensures that a path is prefixed by either package:// or file://
 *
 * Some ROS tools (such as rviz) require even local paths to be explicitely prefixed. Thus
 * - If a path is already prefixed by file:// or package://, leave it unchanged.
 * - If a path has no prefix, assume it is a local path and add the file:// prefix
 */
RBDYN_PARSERS_DLLAPI std::string prefix_path(const std::string & path);

} // namespace parsers

} // namespace rbd
