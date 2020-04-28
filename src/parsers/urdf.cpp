/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/parsers/urdf.h>

#include <algorithm>
#include <ciso646>
#include <cmath>
#include <fstream>
#include <iostream>
#include <locale>
#include <streambuf>
#include <string>
#include <tinyxml2.h>

namespace rbd
{

double attrToDouble(const tinyxml2::XMLElement & dom, const std::string & attr, double def = 0.0)
{
  const char * attrTxt = dom.Attribute(attr.c_str());
  if(attrTxt)
  {
    std::stringstream ss;
    // Every real number in a URDF file needs to be parsed assuming that the
    // decimal point separator is the period, as specified in XML Schema
    // definition of xs:double. The call to imbue ensures that a suitable
    // locale is used, instead of using the current C++ global locale.
    // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
    ss.imbue(std::locale::classic());
    ss << attrTxt;
    double res;
    ss >> res;
    if(!ss.fail())
    {
      return res;
    }
  }
  return def;
}

std::vector<double> attrToList(const tinyxml2::XMLElement & dom,
                               const std::string & attr,
                               const std::vector<double> & def)
{
  std::vector<double> res;
  const char * attrTxt = dom.Attribute(attr.c_str());
  if(attrTxt)
  {
    std::stringstream ss;
    // Every real number in a URDF file needs to be parsed assuming that the
    // decimal point separator is the period, as specified in XML Schema
    // definition of xs:double. The call to imbue ensures that a suitable
    // locale is used, instead of using the current C++ global locale.
    // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
    ss.imbue(std::locale::classic());
    ss << attrTxt;
    double tmp;
    while(ss.good())
    {
      ss >> tmp;
      if(!ss.fail())
      {
        res.push_back(tmp);
      }
    }
  }
  else
  {
    res = def;
  }
  return res;
}

Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom, const std::string & attr, const Eigen::Vector3d & def)
{
  Eigen::Vector3d res = def;
  std::vector<double> vec = attrToList(dom, attr, {res(0), res(1), res(2)});
  if(vec.size() == 3)
  {
    res(0) = vec[0];
    res(1) = vec[1];
    res(2) = vec[2];
  }
  return res;
}

Eigen::Matrix3d RPY(const double & r, const double & p, const double & y)
{
  double ca1 = cos(y);
  double sa1 = sin(y);
  double cb1 = cos(p);
  double sb1 = sin(p);
  double cc1 = cos(r);
  double sc1 = sin(r);
  Eigen::Matrix3d m;
  m << ca1 * cb1, ca1 * sb1 * sc1 - sa1 * cc1, ca1 * sb1 * cc1 + sa1 * sc1, sa1 * cb1, sa1 * sb1 * sc1 + ca1 * cc1,
      sa1 * sb1 * cc1 - ca1 * sc1, -sb1, cb1 * sc1, cb1 * cc1;
  return m.transpose();
}

Eigen::Matrix3d RPY(const std::vector<double> rpy)
{
  if(rpy.size() != 3)
  {
    std::cerr << "Cannot convert RPY vector of size " << rpy.size() << " to matrix" << std::endl;
    throw(std::string("bad vector"));
  }
  return RPY(rpy[0], rpy[1], rpy[2]);
}

inline Eigen::Matrix3d readInertia(const tinyxml2::XMLElement & dom)
{
  Eigen::Matrix3d m;
  m << attrToDouble(dom, "ixx"), attrToDouble(dom, "ixy"), attrToDouble(dom, "ixz"), attrToDouble(dom, "ixy"),
      attrToDouble(dom, "iyy"), attrToDouble(dom, "iyz"), attrToDouble(dom, "ixz"), attrToDouble(dom, "iyz"),
      attrToDouble(dom, "izz");
  return m;
}

rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type, bool hasSphericalSuffix = false)
{
  if(type == "revolute")
    return rbd::Joint::Rev;
  else if(type == "prismatic")
    return rbd::Joint::Prism;
  else if(type == "continuous")
    return rbd::Joint::Rev;
  else if(type == "floating")
  {
    if(hasSphericalSuffix)
      return rbd::Joint::Spherical;
    else
      return rbd::Joint::Free;
  }
  else if(type == "ball")
    return rbd::Joint::Spherical;
  else if(type == "fixed")
    return rbd::Joint::Fixed;
  std::cerr << "Unknown type in URDF " << type << std::endl;
  std::cerr << "Conversion will default to fixed" << std::endl;
  return rbd::Joint::Fixed;
}

sva::PTransformd originFromTag(const tinyxml2::XMLElement * dom)
{
  sva::PTransformd tf = sva::PTransformd::Identity();

  if(dom)
  {
    const tinyxml2::XMLElement * originDom = dom->FirstChildElement("origin");
    if(originDom)
    {
      Eigen::Vector3d T = attrToVector(*originDom, "xyz", Eigen::Vector3d(0, 0, 0));
      Eigen::Matrix3d R = RPY(attrToList(*originDom, "rpy", {0.0, 0.0, 0.0}));
      tf = sva::PTransformd(R, T);
    }
  }

  return tf;
}

sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName)
{
  return originFromTag(root.FirstChildElement(tagName.c_str()));
}

std::string parseMultiBodyGraphFromURDF(ParserResult & res,
                                        const std::string & content,
                                        const std::vector<std::string> & filteredLinksIn,
                                        bool transformInertia,
                                        const std::string & baseLinkIn,
                                        bool withVirtualLinks,
                                        const std::string & sphericalSuffix)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(content.c_str());
  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if(!robot)
  {
    std::cerr << "No robot tag in the URDF, parsing will stop now" << std::endl;
    return "";
  }
  res.name = robot->Attribute("name");
  std::vector<tinyxml2::XMLElement *> links;
  std::vector<std::string> filteredLinks = filteredLinksIn;
  // Extract link elements from the document, remove filtered links
  {
    tinyxml2::XMLElement * link = robot->FirstChildElement("link");
    while(link)
    {
      std::string linkName = link->Attribute("name");
      if(std::find(filteredLinks.begin(), filteredLinks.end(), linkName) == filteredLinks.end())
      {
        if(!withVirtualLinks)
        {
          if(link->FirstChildElement("inertial"))
          {
            links.push_back(link);
          }
          else
          {
            filteredLinks.push_back(linkName);
          }
        }
        else
        {
          links.push_back(link);
        }
      }
      link = link->NextSiblingElement("link");
    }
  }

  if(links.size() == 0)
  {
    std::cerr << "Failed to extract any link information from the URDF, parsing will stop now" << std::endl;
    return "";
  }

  std::string baseLink = baseLinkIn == "" ? links[0]->Attribute("name") : baseLinkIn;

  for(tinyxml2::XMLElement * linkDom : links)
  {
    std::string linkName = linkDom->Attribute("name");
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    std::vector<double> comRPY = {0.0, 0.0, 0.0};
    Eigen::Matrix3d inertia_o = Eigen::Matrix3d::Zero();

    tinyxml2::XMLElement * inertialDom = linkDom->FirstChildElement("inertial");
    bool isVirtual = (inertialDom == 0);
    if(!isVirtual)
    {
      tinyxml2::XMLElement * originDom = inertialDom->FirstChildElement("origin");
      tinyxml2::XMLElement * massDom = inertialDom->FirstChildElement("mass");
      tinyxml2::XMLElement * inertiaDom = inertialDom->FirstChildElement("inertia");

      if(originDom)
      {
        com = attrToVector(*originDom, "xyz", Eigen::Vector3d(0, 0, 0));
        comRPY = attrToList(*originDom, "rpy", {0.0, 0.0, 0.0});
      }
      Eigen::Matrix3d comFrame = RPY(comRPY);
      mass = attrToDouble(*massDom, "value");
      Eigen::Matrix3d inertia = readInertia(*inertiaDom);
      if(transformInertia)
      {
        inertia_o = sva::inertiaToOrigin(inertia, mass, com, comFrame);
      }
      else
      {
        inertia_o = inertia;
      }
    }

    // Parse all visual tags. There may be several per link
    for(tinyxml2::XMLElement * child = linkDom->FirstChildElement("visual"); child != nullptr;
        child = child->NextSiblingElement("visual"))
    {
      Visual v;
      tinyxml2::XMLElement * geometryDom = child->FirstChildElement("geometry");
      if(geometryDom)
      {
        tinyxml2::XMLElement * meshDom = geometryDom->FirstChildElement("mesh");
        if(meshDom)
        {
          v.origin = originFromTag(child);
          v.geometry.type = Geometry::Type::MESH;
          auto mesh = Geometry::Mesh();
          mesh.filename = meshDom->Attribute("filename");
          v.geometry.data = mesh;
          // Optional scale
          mesh.scale = attrToDouble(*meshDom, "scale", 1.0);
        }
        else
        {
          std::cerr << "Warning: only mesh geometry is supported, visual element has been ignored" << std::endl;
        }
        const char * name = child->Attribute("name");
        if(name) v.name = name;
        res.visual[linkName].push_back(v);
      }
    }

    // Parse all collision tags. There may be several per link
    for(tinyxml2::XMLElement * child = linkDom->FirstChildElement("collision"); child != nullptr;
        child = child->NextSiblingElement("collision"))
    {
      Visual v;
      tinyxml2::XMLElement * geometryDom = child->FirstChildElement("geometry");
      if(geometryDom)
      {
        tinyxml2::XMLElement * meshDom = geometryDom->FirstChildElement("mesh");
        if(meshDom)
        {
          v.origin = originFromTag(child);
          v.geometry.type = Geometry::Type::MESH;
          auto mesh = Geometry::Mesh();
          mesh.filename = meshDom->Attribute("filename");
          v.geometry.data = mesh;
          // Optional scale
          double scale = 1.;
          meshDom->QueryDoubleAttribute("scale", &scale);
          mesh.scale = scale;
        }
        else
        {
          tinyxml2::XMLElement * boxDom = geometryDom->FirstChildElement("box");
          if(boxDom)
          {
            v.origin = originFromTag(child);
            v.geometry.type = Geometry::Type::BOX;
            auto box = Geometry::Box();
            box.size = attrToVector(*boxDom, "size", Eigen::Vector3d(0, 0, 0));
            v.geometry.data = box;
          }
          else
          {
            tinyxml2::XMLElement * sphereDom = geometryDom->FirstChildElement("sphere");
            if(sphereDom)
            {
              v.origin = originFromTag(child);
              v.geometry.type = Geometry::Type::SPHERE;
              auto sphere = Geometry::Sphere();
              sphere.radius = sphereDom->DoubleAttribute("radius");
              v.geometry.data = sphere;
            }
            else
            {
              tinyxml2::XMLElement * superellipsoidDom = geometryDom->FirstChildElement("superellipsoid");
              if(superellipsoidDom)
              {
                v.origin = originFromTag(child);
                v.geometry.type = Geometry::Type::SUPERELLIPSOID;
                auto superellipsoid = Geometry::Superellipsoid();
                superellipsoid.size = attrToVector(*superellipsoidDom, "size", Eigen::Vector3d(0, 0, 0));

                double epsilon1 = 1.;
                superellipsoidDom->QueryDoubleAttribute("epsilon1", &epsilon1);
                superellipsoid.epsilon1 = epsilon1;

                double epsilon2 = 1.;
                superellipsoidDom->QueryDoubleAttribute("epsilon2", &epsilon2);
                superellipsoid.epsilon2 = epsilon2;

                v.geometry.data = superellipsoid;
              }
              else
                std::cerr << "Warning: only mesh geometry is supported, collision element has been ignored"
                          << std::endl;
            }
          }
        }
        const char * name = child->Attribute("name");
        if(name) v.name = name;
        res.collision[linkName].push_back(v);
      }
    }

    rbd::Body b(mass, com, inertia_o, linkName);
    res.mbg.addBody(b);
  }

  std::vector<tinyxml2::XMLElement *> joints;
  // Extract joint elements from the document, remove joints that link with filtered links
  {
    tinyxml2::XMLElement * joint = robot->FirstChildElement("joint");
    while(joint)
    {
      std::string parent_link = joint->FirstChildElement("parent")->Attribute("link");
      std::string child_link = joint->FirstChildElement("child")->Attribute("link");
      if(std::find(filteredLinks.begin(), filteredLinks.end(), child_link) == filteredLinks.end()
         && std::find(filteredLinks.begin(), filteredLinks.end(), parent_link) == filteredLinks.end())
      {
        joints.push_back(joint);
      }
      joint = joint->NextSiblingElement("joint");
    }
  }

  for(tinyxml2::XMLElement * jointDom : joints)
  {
    std::string jointName = jointDom->Attribute("name");
    std::string jointType = jointDom->Attribute("type");

    // Static transformation
    sva::PTransformd staticTransform = sva::PTransformd::Identity();
    tinyxml2::XMLElement * originDom = jointDom->FirstChildElement("origin");
    if(originDom)
    {
      Eigen::Vector3d staticT = attrToVector(*originDom, "xyz", Eigen::Vector3d(0, 0, 0));
      Eigen::Matrix3d staticR = RPY(attrToList(*originDom, "rpy", {0.0, 0.0, 0.0}));
      staticTransform = sva::PTransformd(staticR, staticT);
    }

    // Read the joint axis
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    tinyxml2::XMLElement * axisDom = jointDom->FirstChildElement("axis");
    if(axisDom)
    {
      axis = attrToVector(*axisDom, "xyz").normalized();
    }
    rbd::Joint::Type type = rbdynFromUrdfJoint(
        jointType, (jointName.length() >= sphericalSuffix.length()
                    && jointName.substr(jointName.length() - sphericalSuffix.length(), sphericalSuffix.length())
                           == sphericalSuffix));

    tinyxml2::XMLElement * parentDom = jointDom->FirstChildElement("parent");
    std::string jointParent = parentDom->Attribute("link");

    tinyxml2::XMLElement * childDom = jointDom->FirstChildElement("child");
    std::string jointChild = childDom->Attribute("link");

    rbd::Joint j(type, axis, true, jointName);

    // Check if this is a mimic joint
    tinyxml2::XMLElement * mimicDom = jointDom->FirstChildElement("mimic");
    if(mimicDom)
    {
      std::string mimicJoint = mimicDom->Attribute("joint");
      j.makeMimic(mimicJoint, attrToDouble(*mimicDom, "multiplier", 1.0), attrToDouble(*mimicDom, "offset"));
    }

    res.mbg.addJoint(j);

    res.mbg.linkBodies(jointParent, staticTransform, jointChild, sva::PTransformd::Identity(), jointName);

    // Articular limit
    std::vector<double> lower(static_cast<size_t>(j.dof()), -INFINITY);
    std::vector<double> upper(static_cast<size_t>(j.dof()), INFINITY);
    std::vector<double> effort(static_cast<size_t>(j.dof()), INFINITY);
    std::vector<double> velocity(static_cast<size_t>(j.dof()), INFINITY);

    tinyxml2::XMLElement * limitDom = jointDom->FirstChildElement("limit");
    if(limitDom && j.type() != rbd::Joint::Fixed)
    {
      if(jointType != "continuous")
      {
        lower = attrToList(*limitDom, "lower");
        upper = attrToList(*limitDom, "upper");
      }
      effort = attrToList(*limitDom, "effort");
      velocity = attrToList(*limitDom, "velocity");
    }
    auto check_limit = [&j](const std::string & name, const std::vector<double> & limit) {
      if(limit.size() != static_cast<size_t>(j.dof()))
      {
        std::cerr << "Joint " << name << " limit for " << j.name() << ": size missmatch, expected: " << j.dof()
                  << ", got: " << limit.size() << std::endl;
      }
    };
    check_limit("lower", lower);
    check_limit("upper", upper);
    check_limit("effort", effort);
    check_limit("velocity", velocity);
    res.limits.lower[jointName] = lower;
    res.limits.upper[jointName] = upper;
    res.limits.torque[jointName] = effort;
    res.limits.velocity[jointName] = velocity;
  }

  return baseLink;
}

ParserResult from_urdf(const std::string & content,
                       bool fixed,
                       const std::vector<std::string> & filteredLinksIn,
                       bool transformInertia,
                       const std::string & baseLinkIn,
                       bool withVirtualLinks,
                       const std::string & sphericalSuffix)
{
  ParserResult res;

  std::string baseLink = parseMultiBodyGraphFromURDF(res, content, filteredLinksIn, transformInertia, baseLinkIn,
                                                     withVirtualLinks, sphericalSuffix);

  res.mb = res.mbg.makeMultiBody(baseLink, fixed);
  res.mbc = rbd::MultiBodyConfig(res.mb);
  res.mbc.zero(res.mb);

  rbd::forwardKinematics(res.mb, res.mbc);
  rbd::forwardVelocity(res.mb, res.mbc);

  return res;
}

ParserResult from_urdf_file(const std::string & file_path,
                            bool fixed,
                            const std::vector<std::string> & filteredLinksIn,
                            bool transformInertia,
                            const std::string & baseLinkIn,
                            bool withVirtualLinks,
                            const std::string & sphericalSuffix)
{
  std::ifstream file(file_path);
  if(!file.is_open())
  {
    throw std::runtime_error("URDF: Can't open " + file_path + " file for reading");
  }
  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return from_urdf(content, fixed, filteredLinksIn, transformInertia, baseLinkIn, withVirtualLinks, sphericalSuffix);
}

} // namespace rbd
