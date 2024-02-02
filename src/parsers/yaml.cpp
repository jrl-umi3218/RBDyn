/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/parsers/yaml.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <yaml-cpp/yaml.h>

#ifndef WIN32
#  include <cxxabi.h>
#endif

/** value from boost/math/constants */
#ifndef M_PI
#  define M_PI 3.141592653589793238462643383279502884e+00
#endif

namespace rbd
{

namespace parsers
{

namespace
{

static inline std::string demangle(const char * name)
{
#ifndef WIN32
  int status = 0;
  std::unique_ptr<char, void (*)(void *)> res{abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
  return status == 0 ? res.get() : name;
#else
  return name;
#endif
}

} // namespace

RBDynFromYAML::RBDynFromYAML(const std::string & input,
                             ParserInput input_type,
                             bool fixed,
                             const std::vector<std::string> & filtered_links,
                             bool transform_inertia,
                             const std::string & base_link,
                             bool with_virtual_links,
                             const std::string & spherical_suffix,
                             bool remove_filtered_links)
: verbose_(false), transform_inertia_(transform_inertia), link_idx_(1), joint_idx_(1), filtered_links_(filtered_links),
  remove_filtered_links_(remove_filtered_links), with_virtual_links_(with_virtual_links),
  spherical_suffix_(spherical_suffix)
{
  joint_types_ = std::map<std::string, rbd::Joint::Type>{
      {"revolute", rbd::Joint::Rev},        {"continuous", rbd::Joint::Rev}, {"prismatic", rbd::Joint::Prism},
      {"spherical", rbd::Joint::Spherical}, {"ball", rbd::Joint::Spherical}, {"free", rbd::Joint::Free},
      {"fixed", rbd::Joint::Fixed}};

  YAML::Node config;
  if(input_type == ParserInput::File)
  {
    config = YAML::LoadFile(input);
  }
  else
  {
    config = YAML::Load(input);
  }

  YAML::Node robot = config["robot"];
  if(!robot)
  {
    throw std::runtime_error("YAML: missing 'robot' root element");
  }

  res.name = robot["name"].as<std::string>();
  if(verbose_)
  {
    std::cout << "Robot name: " << res.name << std::endl;
  }

  YAML::Node materials = robot["materials"];
  if(materials)
  {
    for(const auto & material : materials)
    {
      parseMaterial(material);
    }
  }

  YAML::Node links = robot["links"];
  if(!links)
  {
    throw std::runtime_error("YAML: missing 'robot->links' element");
  }

  YAML::Node angles_in_degrees = robot["anglesInDegrees"];
  if(!angles_in_degrees)
  {
    throw std::runtime_error("YAML: missing 'robot->anglesInDegrees: true/false' element");
  }
  else
  {
    angles_in_degrees_ = angles_in_degrees.as<bool>();
  }

  for(const auto & link : links)
  {
    parseLink(link);
  }

  YAML::Node joints = robot["joints"];
  if(!joints)
  {
    throw std::runtime_error("YAML: missing 'robot->joints' element");
  }
  for(const auto & joint : joints)
  {
    parseJoint(joint);
  }

  if(!base_link.empty())
  {
    base_link_ = base_link;
  }
  if(verbose_)
  {
    std::cout << "base_link: " << base_link_ << std::endl;
  }

  res.mb = res.mbg.makeMultiBody(base_link_, fixed);

  res.mbc = rbd::MultiBodyConfig(res.mb);
  res.mbc.zero(res.mb);

  rbd::forwardKinematics(res.mb, res.mbc);
  rbd::forwardVelocity(res.mb, res.mbc);
}

Eigen::Matrix3d RBDynFromYAML::makeInertia(double ixx, double iyy, double izz, double iyz, double ixz, double ixy)
{
  Eigen::Matrix3d inertia;
  inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  return inertia;
}

Eigen::Matrix3d RBDynFromYAML::MatrixFromRPY(double r, double p, double y)
{
  Eigen::Quaterniond q = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d m;
  m = q;
  return m.transpose().eval();
}

void RBDynFromYAML::parseFrame(const YAML::Node & frame,
                               const std::string & name,
                               Eigen::Vector3d & xyz,
                               Eigen::Vector3d & rpy)
{
  xyz.setZero();
  rpy.setZero();
  if(frame)
  {
    auto xyz_node = frame["xyz"];
    if(xyz_node)
    {
      auto xyz_data = xyz_node.as<std::vector<double>>();
      if(xyz_data.size() != 3)
      {
        throw std::runtime_error("YAML: Invalid array size (" + name + "->intertial->frame->xyz");
      }
      std::copy_n(std::begin(xyz_data), 3, xyz.data());
    }
    auto rpy_node = frame["rpy"];
    if(rpy_node)
    {
      auto rpy_data = rpy_node.as<std::vector<double>>();
      if(rpy_data.size() != 3)
      {
        throw std::runtime_error("YAML: Invalid array size (" + name + "->intertial->frame->rpy");
      }
      std::copy_n(std::begin(rpy_data), 3, rpy.data());
      if(frame["anglesInDegrees"].as<bool>(angles_in_degrees_))
      {
        rpy *= M_PI / 180.;
      }
    }
  }
}

void RBDynFromYAML::parseInertia(const YAML::Node & inertia, Eigen::Matrix3d & inertia_mat)
{
  inertia_mat.setIdentity();
  if(inertia)
  {
    inertia_mat =
        makeInertia(inertia["Ixx"].as<double>(1.), inertia["Iyy"].as<double>(1.), inertia["Izz"].as<double>(1.),
                    inertia["Iyz"].as<double>(0.), inertia["Ixz"].as<double>(0.), inertia["Ixy"].as<double>(0.));
  }
}

void RBDynFromYAML::parseInertial(const YAML::Node & inertial,
                                  const std::string & name,
                                  double & mass,
                                  Eigen::Vector3d & xyz,
                                  Eigen::Vector3d & rpy,
                                  Eigen::Matrix3d & inertia)
{
  mass = 0.;
  xyz.setZero();
  rpy.setZero();
  inertia.setZero();
  if(inertial)
  {
    mass = inertial["mass"].as<double>(mass);
    parseFrame(inertial["frame"], name, xyz, rpy);
    parseInertia(inertial["inertia"], inertia);

    if(transform_inertia_)
    {
      Eigen::Matrix3d rot = MatrixFromRPY(rpy[0], rpy[1], rpy[2]);
      inertia = sva::inertiaToOrigin(inertia, mass, xyz, rot);
    }
  }
}

void RBDynFromYAML::parseMaterial(const YAML::Node & material)
{
  if(!material)
  {
    return;
  }
  auto name = material["name"].as<std::string>("");
  if(name.empty())
  {
    return;
  }
  parseMaterial(material, materials_[name]);
}

void RBDynFromYAML::parseMaterial(const YAML::Node & material, Material & out, bool cache)
{
  if(!material)
  {
    return;
  }
  auto name = material["name"].as<std::string>("");
  if(name.empty())
  {
    return;
  }
  if(cache && materials_.count(name))
  {
    out = materials_[name];
  }
  auto color = material["color"];
  if(color)
  {
    auto color_data = color["rgba"].as<std::vector<double>>();
    if(color_data.size() != 4)
    {
      std::cerr << "YAML: Invalid rgba size in color element (" << color_data.size() << ") in material " << name
                << "\nThis material will be ignored\n";
      return;
    }
    out.type = Material::Type::COLOR;
    const auto & c = color_data;
    out.data = Material::Color{c[0], c[1], c[2], c[3]};
    return;
  }
  auto texture = material["texture"];
  if(texture)
  {
    auto filename = texture["filename"].as<std::string>("");
    if(filename.empty())
    {
      std::cerr << "YAML: Empty filename in texture element in material " << name
                << "\nThis material will be ignored\n";
      return;
    }
    out.type = Material::Type::TEXTURE;
    out.data = Material::Texture{filename};
    return;
  }
  std::cerr << "YAML: material " << name << " has no color or texture element, it will be ignored\n";
  return;
}

bool RBDynFromYAML::parseGeometry(const YAML::Node & geometry, Geometry & data)
{
  bool has_geometry = false;
  data = Geometry();
  if(geometry)
  {
    for(YAML::const_iterator geometry_type = geometry.begin(); geometry_type != geometry.end(); ++geometry_type)
    {
      auto type = geometry_type->first.as<std::string>("");
      if(type == "mesh")
      {
        data.type = Geometry::Type::MESH;

        const auto & mesh = geometry_type->second;
        auto mesh_data = Geometry::Mesh();
        try
        {
          mesh_data.filename = mesh["filename"].as<std::string>();
        }
        catch(...)
        {
          throw std::runtime_error("YAML: a mesh geometry requires a filename field.");
        }
        auto maybeScaleV = mesh["scale"].as<std::vector<double>>(std::vector<double>{1.0});
        if(maybeScaleV.size() == 3)
        {
          mesh_data.scaleV = Eigen::Map<Eigen::Vector3d>(maybeScaleV.data(), 3);
        }
        else
        {
          assert(maybeScaleV.size() == 1);
          mesh_data.scaleV.setConstant(maybeScaleV[0]);
        }
        has_geometry = true;
        data.data = mesh_data;
      }
      else if(type == "box")
      {
        data.type = Geometry::Type::BOX;

        const auto & box = geometry_type->second;
        auto box_data = Geometry::Box();
        try
        {
          box_data.size = Eigen::Vector3d(box["size"].as<std::vector<double>>().data());
          if(box_data.size.size() != 3)
          {
            throw std::runtime_error("YAML: Invalid box size, should have 3 components (x, y, z)");
          }
        }
        catch(...)
        {
          throw std::runtime_error("YAML: a box geometry requires a size field.");
        }
        has_geometry = true;
        data.data = box_data;
      }
      else if(type == "cylinder")
      {
        data.type = Geometry::Type::CYLINDER;

        const auto & cylinder = geometry_type->second;
        auto cylinder_data = Geometry::Cylinder();
        try
        {
          cylinder_data.radius = cylinder["radius"].as<double>();
          cylinder_data.length = cylinder["length"].as<double>();
        }
        catch(...)
        {
          throw std::runtime_error("YAML: a cylinder geometry requires radius and length fields.");
        }
        has_geometry = true;
        data.data = cylinder_data;
      }
      else if(type == "sphere")
      {
        data.type = Geometry::Type::SPHERE;

        const auto & sphere = geometry_type->second;
        auto sphere_data = Geometry::Sphere();
        try
        {
          sphere_data.radius = sphere["radius"].as<double>();
        }
        catch(...)
        {
          throw std::runtime_error("YAML: a sphere geometry requires a radius field.");
        }
        has_geometry = true;
        data.data = sphere_data;
      }
      else if(type == "superellipsoid")
      {
        data.type = Geometry::Type::SUPERELLIPSOID;

        const auto & superellipsoid = geometry_type->second;
        auto superellipsoid_data = Geometry::Superellipsoid();
        try
        {
          superellipsoid_data.size = Eigen::Vector3d(superellipsoid["size"].as<std::vector<double>>().data());
          if(superellipsoid_data.size.size() != 3)
          {
            throw std::runtime_error("YAML: Invalid superellipsoid size, should have 3 components (x, y, z)");
          }
          superellipsoid_data.epsilon1 = superellipsoid["epsilon1"].as<double>();
          superellipsoid_data.epsilon2 = superellipsoid["epsilon2"].as<double>();
        }
        catch(...)
        {
          throw std::runtime_error("YAML: a sphere geometry requires size, epsilon1 and epsilon2 fields.");
        }
        has_geometry = true;
        data.data = superellipsoid_data;
      }
      else
      {
        throw std::runtime_error("YAML: unkown geometry type '" + type
                                 + "'.Supported geometries are mesh, box, cylinder and sphere.");
      }
    }
  }
  return has_geometry;
}

void RBDynFromYAML::parseVisuals(const YAML::Node & visuals,
                                 std::map<std::string, std::vector<Visual>> & data,
                                 const std::string & name)
{
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;
  for(const auto & visual : visuals)
  {
    Visual v;
    v.name = visual["name"].as<std::string>("");

    parseFrame(visual["frame"], v.name, xyz, rpy);
    v.origin.rotation() = MatrixFromRPY(rpy[0], rpy[1], rpy[2]);
    v.origin.translation() = xyz;

    parseMaterial(visual["material"], v.material);

    if(parseGeometry(visual["geometry"], v.geometry))
    {
      data[name].push_back(v);
    }
  }
}

void RBDynFromYAML::parseLink(const YAML::Node & link)
{
  std::string name = link["name"].as<std::string>("link" + std::to_string(link_idx_));

  if(verbose_)
  {
    std::cout << "Parsing link: " << name << std::endl;
  }

  if(!with_virtual_links_ && !link["inertial"])
  {
    removed_links_.push_back(name);
    return;
  }

  if(std::find(filtered_links_.begin(), filtered_links_.end(), name) != filtered_links_.end())
  {
    if(remove_filtered_links_)
    {
      removed_links_.push_back(name);
      return;
    }
    else
    {
      fixed_links_.push_back(name);
    }
  }

  if(base_link_.empty())
  {
    base_link_ = name;
  }

  double mass;
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;
  Eigen::Matrix3d inertia;
  parseInertial(link["inertial"], name, mass, xyz, rpy, inertia);

  res.mbg.addBody(rbd::Body(mass, xyz, inertia, name));

  parseVisuals(link["visual"], res.visual, name);
  parseVisuals(link["collision"], res.collision, name);

  if(verbose_)
  {
    std::cout << "\tmass: " << mass << '\n';
    std::cout << "\txyz: " << xyz.transpose() << '\n';
    std::cout << "\trpy: " << rpy.transpose() << '\n';
    std::cout << "\tinertia:\n";
    std::cout << "\t\t" << inertia(0, 0) << '\t' << inertia(0, 1) << '\t' << inertia(0, 2) << '\n';
    std::cout << "\t\t" << inertia(1, 0) << '\t' << inertia(1, 1) << '\t' << inertia(1, 2) << '\n';
    std::cout << "\t\t" << inertia(2, 0) << '\t' << inertia(2, 1) << '\t' << inertia(2, 2) << '\n';
    auto visuals_it = res.visual.find(name);
    if(visuals_it != res.visual.end())
    {
      for(const auto & visual : visuals_it->second)
      {
        std::cout << "\tvisual:\n";
        std::cout << "\t\tname: " << visual.name << '\n';
        std::cout << "\t\torigin: " << visual.origin.translation().transpose() << ", "
                  << visual.origin.rotation().eulerAngles(0, 1, 2).transpose() << '\n';
        std::cout << "\t\ttype: " << demangle(visual.geometry.data.type().name()) << '\n';
      }
    }
  }

  ++link_idx_;
}

bool RBDynFromYAML::parseJointType(const YAML::Node & type,
                                   const std::string & name,
                                   rbd::Joint::Type & joint_type,
                                   std::string & type_name,
                                   bool force_fixed)
{
  if(force_fixed)
  {
    joint_type = rbd::Joint::Type::Fixed;
    type_name = "fixed";
    return false;
  }
  if(type)
  {
    type_name = type.as<std::string>();

    if(type_name == "floating")
    {
      bool hasSphericalSuffix =
          name.length() >= spherical_suffix_.length()
          && name.substr(name.length() - spherical_suffix_.length(), spherical_suffix_.length()) == spherical_suffix_;
      if(hasSphericalSuffix)
      {
        type_name = "spherical";
      }
      else
      {
        type_name = "free";
      }
    }

    try
    {
      joint_type = joint_types_.at(type_name);
    }
    catch(std::exception &)
    {
      std::string possibilities;
      size_t type_count = joint_types_.size();
      size_t idx = 0;
      for(const auto & t : joint_types_)
      {
        possibilities.append(t.first);
        if(idx++ < type_count - 1)
        {
          possibilities.append(",");
        }
      }
      throw std::runtime_error("YAML: unkown joint type " + type_name + " (" + name
                               + "). Possible values are: " + possibilities);
    }

    return type_name == "continuous";
  }
  else
  {
    type_name = "revolute";
    joint_type = rbd::Joint::Rev;
    return false;
  }
}

void RBDynFromYAML::parseJointAxis(const YAML::Node & axis, const std::string & name, Eigen::Vector3d & joint_axis)
{
  if(axis)
  {
    auto axis_data = axis.as<std::vector<double>>();
    if(axis_data.size() != 3)
    {
      throw std::runtime_error("YAML: Invalid array size (" + name + "->intertial->frame->axis");
    }
    std::copy_n(std::begin(axis_data), 3, joint_axis.data());
  }
  else
  {
    joint_axis = Eigen::Vector3d(0., 0., 1.);
  }
}

void RBDynFromYAML::parseJointLimits(const YAML::Node & limits,
                                     const std::string & name,
                                     const rbd::Joint & joint,
                                     bool is_continuous)
{
  auto dof_count = static_cast<size_t>(joint.dof());
  auto infinity = std::numeric_limits<double>::infinity();
  std::vector<double> lower(dof_count, -infinity);
  std::vector<double> upper(dof_count, infinity);
  std::vector<double> effort(dof_count, infinity);
  std::vector<double> velocity(dof_count, infinity);

  if(limits)
  {
    if(dof_count > 1)
    {
      if(!is_continuous)
      {
        lower = limits["lower"].as<std::vector<double>>(lower);
        upper = limits["upper"].as<std::vector<double>>(upper);
      }
      effort = limits["effort"].as<std::vector<double>>(effort);
      velocity = limits["velocity"].as<std::vector<double>>(velocity);
    }
    else if(dof_count == 1)
    {
      if(!is_continuous)
      {
        lower[0] = limits["lower"].as<double>(-infinity);
        upper[0] = limits["upper"].as<double>(infinity);
      }
      effort[0] = limits["effort"].as<double>(infinity);
      velocity[0] = limits["velocity"].as<double>(infinity);
    }
    auto check_limit = [&joint](const std::string & name, const std::vector<double> & limit)
    {
      if(limit.size() != static_cast<size_t>(joint.dof()))
      {
        std::cerr << "YAML: joint " << name << " limit for " << joint.name()
                  << ": size missmatch, expected: " << joint.dof() << ", got: " << limit.size() << std::endl;
      }
    };
    check_limit("lower", lower);
    check_limit("upper", upper);
    check_limit("effort", effort);
    check_limit("velocity", velocity);

    if(limits["anglesInDegrees"].as<bool>(angles_in_degrees_))
    {
      for(size_t i = 0; i < dof_count; ++i)
      {
        lower[i] *= M_PI / 180.;
        upper[i] *= M_PI / 180.;
        velocity[i] *= M_PI / 180.;
      }
    }
  }

  res.limits.lower[name] = lower;
  res.limits.upper[name] = upper;
  res.limits.torque[name] = effort;
  res.limits.velocity[name] = velocity;
}

void RBDynFromYAML::parseJoint(const YAML::Node & joint)
{
  std::string name = joint["name"].as<std::string>("joint" + std::to_string(joint_idx_));
  if(verbose_)
  {
    std::cout << "Parsing joint: " << name << std::endl;
  }

  std::string parent = joint["parent"].as<std::string>("link" + std::to_string(joint_idx_));
  std::string child = joint["child"].as<std::string>("link" + std::to_string(joint_idx_ + 1));
  auto is_removed = [&](const std::string & link)
  { return std::find(removed_links_.begin(), removed_links_.end(), link) != removed_links_.end(); };
  if(is_removed(child) || is_removed(parent))
  {
    return;
  }
  auto is_fixed = [&](const std::string & link)
  { return std::find(fixed_links_.begin(), fixed_links_.end(), link) != fixed_links_.end(); };
  std::string type_name;
  rbd::Joint::Type type;
  Eigen::Vector3d axis;
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;

  bool is_continuous = parseJointType(joint["type"], name, type, type_name, is_fixed(child));
  parseJointAxis(joint["axis"], name, axis);
  parseFrame(joint["frame"], name, xyz, rpy);

  rbd::Joint j(type, axis, true, name);

  parseJointLimits(joint["limits"], name, j, is_continuous);

  if(verbose_)
  {
    std::cout << "\ttype: " << type_name << '\n';
    std::cout << "\txyz: " << xyz.transpose() << '\n';
    std::cout << "\trpy: " << rpy.transpose() << '\n';
    std::cout << "\tparent: " << parent << '\n';
    std::cout << "\tchild: " << child << '\n';
    std::cout << "\tlimits:\n";
    std::cout << "\t\tlower: [";
    std::copy(res.limits.lower[name].begin(), res.limits.lower[name].end(),
              std::ostream_iterator<double>(std::cout, ", "));
    std::cout << "\b\b]\n";
    std::cout << "\t\tupper: [";
    std::copy(res.limits.upper[name].begin(), res.limits.upper[name].end(),
              std::ostream_iterator<double>(std::cout, ", "));
    std::cout << "\b\b]\n";
    std::cout << "\t\tvelocity: [";
    std::copy(res.limits.velocity[name].begin(), res.limits.velocity[name].end(),
              std::ostream_iterator<double>(std::cout, ", "));
    std::cout << "\b\b]\n";
    std::cout << "\t\ttorque: [";
    std::copy(res.limits.torque[name].begin(), res.limits.torque[name].end(),
              std::ostream_iterator<double>(std::cout, ", "));
    std::cout << "\b\b]\n";
  }

  res.mbg.addJoint(j);
  Eigen::Matrix3d rot = MatrixFromRPY(rpy[0], rpy[1], rpy[2]);
  res.mbg.linkBodies(parent, sva::PTransformd(rot, xyz), child, sva::PTransformd::Identity(), name);

  ++joint_idx_;
}

ParserResult from_yaml(const std::string & content,
                       bool fixed,
                       const std::vector<std::string> & filteredLinksIn,
                       bool transformInertia,
                       const std::string & baseLinkIn,
                       bool withVirtualLinks,
                       const std::string & sphericalSuffix)
{
  return RBDynFromYAML(content, ParserInput::Description, fixed, filteredLinksIn, transformInertia, baseLinkIn,
                       withVirtualLinks, sphericalSuffix)
      .result();
}

ParserResult from_yaml(const std::string & content, const ParserParameters & params)
{
  return RBDynFromYAML(content, ParserInput::Description, params.fixed_, params.filtered_links_,
                       params.transform_inertia_, params.base_link_, !params.remove_virtual_links_,
                       params.spherical_suffix_, params.remove_filtered_links_)
      .result();
}

ParserResult from_yaml_file(const std::string & file_path,
                            bool fixed,
                            const std::vector<std::string> & filteredLinksIn,
                            bool transformInertia,
                            const std::string & baseLinkIn,
                            bool withVirtualLinks,
                            const std::string & sphericalSuffix)
{
  return RBDynFromYAML(file_path, ParserInput::File, fixed, filteredLinksIn, transformInertia, baseLinkIn,
                       withVirtualLinks, sphericalSuffix)
      .result();
}

ParserResult from_yaml_file(const std::string & file_path, const ParserParameters & params)
{
  return RBDynFromYAML(file_path, ParserInput::File, params.fixed_, params.filtered_links_, params.transform_inertia_,
                       params.base_link_, !params.remove_virtual_links_, params.spherical_suffix_,
                       params.remove_filtered_links_)
      .result();
}

} // namespace parsers

} // namespace rbd
