#include <RBDyn/parsers/urdf.h>

#include <locale>
#include <tinyxml2.h>

namespace rbd
{
namespace parsers
{

std::string to_urdf(const ParserResult & res)
{
  using namespace tinyxml2;

  XMLDocument doc;
  auto robot = doc.NewElement("robot");
  robot->SetAttribute("name", res.name.c_str());

  auto set_double = [](XMLElement * e, const char * name, double value)
  {
    std::stringstream ss;
    ss.imbue(std::locale::classic());
    ss << value;
    e->SetAttribute(name, ss.str().c_str());
  };

  auto set_vec3d = [](XMLElement * e, const char * name, Eigen::Ref<const Eigen::Vector3d> xyz)
  {
    std::stringstream ss;
    ss.imbue(std::locale::classic());
    Eigen::IOFormat f(Eigen::StreamPrecision, Eigen::DontAlignCols);
    ss << xyz.transpose().format(f);
    e->SetAttribute(name, ss.str().c_str());
  };

  auto set_origin_from_ptransform = [&](XMLElement * node, const sva::PTransformd & X)
  {
    const auto & xyz = X.translation();
    const auto rpy = X.rotation().transpose().eulerAngles(0, 1, 2);
    if(!xyz.isZero() || !rpy.isZero())
    {
      auto origin_node = doc.NewElement("origin");
      if(!xyz.isZero())
      {
        set_vec3d(origin_node, "xyz", xyz);
      }
      if(!rpy.isZero())
      {
        set_vec3d(origin_node, "rpy", rpy);
      }
      node->InsertEndChild(origin_node);
    }
  };

  // Links
  for(const auto & link : res.mb.bodies())
  {
    auto node = doc.NewElement("link");
    node->SetAttribute("name", link.name().c_str());

    // Inertial
    const auto has_mass = link.inertia().mass() > 0.;
    const auto has_momentum = !link.inertia().momentum().isZero();
    const auto has_inertia = !link.inertia().inertia().isZero();
    if(has_mass || has_momentum || has_inertia)
    {

      auto inertial_node = doc.NewElement("inertial");

      const auto com = [&]() -> Eigen::Vector3d
      {
        if(link.inertia().mass() > 0.)
        {
          return link.inertia().momentum() / link.inertia().mass();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }();
      if(!com.isZero())
      {
        auto origin_node = doc.NewElement("origin");
        set_vec3d(origin_node, "xyz", com);
        inertial_node->InsertEndChild(origin_node);
      }

      if(link.inertia().mass() > 0.)
      {
        auto mass_node = doc.NewElement("mass");
        set_double(mass_node, "value", link.inertia().mass());
        inertial_node->InsertEndChild(mass_node);
      }

      if(!link.inertia().inertia().isZero())
      {
        //! Transform the inertia before writing it so that it can be later read using the 'transformInertia' parameter
        //! to false (default value)
        const auto inertia = sva::inertiaToOrigin(link.inertia().inertia(), -link.inertia().mass(), com,
                                                  Eigen::Matrix3d::Identity().eval());

        auto inertia_node = doc.NewElement("inertia");
        set_double(inertia_node, "ixx", inertia(0, 0));
        set_double(inertia_node, "ixy", inertia(0, 1));
        set_double(inertia_node, "ixz", inertia(0, 2));
        set_double(inertia_node, "iyy", inertia(1, 1));
        set_double(inertia_node, "iyz", inertia(1, 2));
        set_double(inertia_node, "izz", inertia(2, 2));
        inertial_node->InsertEndChild(inertia_node);
      }

      node->InsertEndChild(inertial_node);
    }

    auto generate_visual = [&](const char * type, const std::map<std::string, std::vector<Visual>> & visuals)
    {
      auto visuals_it = visuals.find(link.name());
      if(visuals_it == visuals.end())
      {
        return;
      }
      for(size_t i = 0; i < visuals_it->second.size(); ++i)
      {
        const auto & visual = visuals_it->second[i];
        if(visual.geometry.type == Geometry::Type::UNKNOWN)
        {
          continue;
        }

        auto visual_node = doc.NewElement(type);

        set_origin_from_ptransform(visual_node, visual.origin);

        const auto & material = visual.material;
        if(material.type != Material::Type::NONE)
        {
          auto material_node = doc.NewElement("material");
          material_node->SetAttribute("name", ("material_" + link.name() + "_" + std::to_string(i)).c_str());
          if(material.type == Material::Type::COLOR)
          {
            const auto & c = boost::get<Material::Color>(material.data);
            auto color_node = doc.NewElement("color");
            std::stringstream ss;
            ss.imbue(std::locale::classic());
            ss << c.r << " " << c.g << " " << c.b << " " << c.a;
            color_node->SetAttribute("rgba", ss.str().c_str());
            material_node->InsertEndChild(color_node);
          }
          else if(material.type == Material::Type::TEXTURE)
          {
            const auto & texture = boost::get<Material::Texture>(material.data);
            auto texture_node = doc.NewElement("texture");
            texture_node->SetAttribute("filename", prefix_path(texture.filename).c_str());
            material_node->InsertEndChild(texture_node);
          }
          visual_node->InsertEndChild(material_node);
        }

        auto geometry_node = doc.NewElement("geometry");
        switch(visual.geometry.type)
        {
          case Geometry::Type::BOX:
          {
            auto node = doc.NewElement("box");
            const auto box = boost::get<Geometry::Box>(visual.geometry.data);
            set_vec3d(node, "size", box.size);
            geometry_node->InsertEndChild(node);
          }
          break;
          case Geometry::Type::CYLINDER:
          {
            auto node = doc.NewElement("cylinder");
            const auto cylinder = boost::get<Geometry::Cylinder>(visual.geometry.data);
            set_double(node, "radius", cylinder.radius);
            set_double(node, "length", cylinder.length);
            geometry_node->InsertEndChild(node);
          }
          break;
          case Geometry::Type::MESH:
          {
            auto node = doc.NewElement("mesh");
            const auto mesh = boost::get<Geometry::Mesh>(visual.geometry.data);
            node->SetAttribute("filename", prefix_path(mesh.filename).c_str());
            set_vec3d(node, "scale", mesh.scaleV);
            geometry_node->InsertEndChild(node);
          }
          break;
          case Geometry::Type::SPHERE:
          {
            auto node = doc.NewElement("sphere");
            const auto sphere = boost::get<Geometry::Sphere>(visual.geometry.data);
            set_double(node, "radius", sphere.radius);
            geometry_node->InsertEndChild(node);
          }
          break;
          case Geometry::Type::SUPERELLIPSOID:
          {
            auto node = doc.NewElement("superellipsoid");
            const auto superellipsoid = boost::get<Geometry::Superellipsoid>(visual.geometry.data);
            set_vec3d(node, "size", superellipsoid.size);
            set_double(node, "epsilon1", superellipsoid.epsilon1);
            set_double(node, "epsilon2", superellipsoid.epsilon2);
            geometry_node->InsertEndChild(node);
          }
          break;
          case Geometry::Type::UNKNOWN:
            break;
        }
        visual_node->InsertEndChild(geometry_node);

        node->InsertEndChild(visual_node);
      }
    };

    generate_visual("visual", res.visual);
    generate_visual("collision", res.collision);

    robot->InsertEndChild(node);
  }

  auto is_continuous = [&](const rbd::Joint & joint) -> bool
  {
    const bool has_upper_limit = res.limits.upper.count(joint.name()) > 0;
    const bool has_lower_limit = res.limits.lower.count(joint.name()) > 0;
    if(!has_upper_limit && !has_lower_limit)
    {
      return true;
    }
    else if(has_upper_limit && res.limits.upper.at(joint.name())[0] != std::numeric_limits<double>::infinity())
    {
      return false;
    }
    else if(has_lower_limit && res.limits.lower.at(joint.name())[0] != -std::numeric_limits<double>::infinity())
    {
      return false;
    }
    else
    {
      return true;
    }
  };

  auto set_vector = [](XMLElement * e, const char * name, const std::vector<double> & v)
  {
    std::stringstream ss;
    ss.imbue(std::locale::classic());
    for(size_t i = 0; i < v.size(); i++)
    {
      ss << v[i];
      if(i != v.size() - 1)
      {
        ss << ' ';
      }
    }
    e->SetAttribute(name, ss.str().c_str());
  };

  auto has_limits = [&](const Joint & joint)
  {
    auto check = [](const Joint & joint, const std::map<std::string, std::vector<double>> & limits)
    {
      auto it = limits.find(joint.name());
      if(it != limits.end())
      {
        return !it->second.empty();
      }
      else
      {
        return false;
      }
    };
    return check(joint, res.limits.lower) && check(joint, res.limits.upper) && check(joint, res.limits.velocity)
           && check(joint, res.limits.torque);
  };

  auto set_limit = [&](XMLElement * e, const Joint & joint, const char * name,
                       const std::map<std::string, std::vector<double>> & limits)
  {
    auto it = limits.find(joint.name());
    if(it != limits.end())
    {
      set_vector(e, name, it->second);
    }
  };

  // Joints
  for(const auto & joint : res.mb.joints())
  {
    // Skip the root joint
    if(res.mb.jointIndexByName(joint.name()) == 0)
    {
      continue;
    }
    auto node = doc.NewElement("joint");
    node->SetAttribute("name", joint.name().c_str());
    switch(joint.type())
    {
      case Joint::Type::Rev:
        if(is_continuous(joint))
        {
          node->SetAttribute("type", "continuous");
        }
        else
        {
          node->SetAttribute("type", "revolute");
        }
        break;
      case Joint::Type::Prism:
        node->SetAttribute("type", "prismatic");
        break;
      case Joint::Type::Spherical:
        throw std::invalid_argument("URDF: Spherical is an unsupported joint type");
        break;
      case Joint::Type::Planar:
        node->SetAttribute("type", "planar");
        break;
      case Joint::Type::Cylindrical:
        throw std::invalid_argument("URDF: Cylindrical is an unsupported joint type");
        break;
      case Joint::Type::Free:
        node->SetAttribute("type", "floating");
        break;
      case Joint::Type::Fixed:
        node->SetAttribute("type", "fixed");
        break;
    }

    auto index = res.mb.jointIndexByName(joint.name());
    const auto pred = res.mb.predecessor(index);
    auto parent_node = doc.NewElement("parent");
    const auto & parent = [&]()
    {
      if(pred != -1)
      {
        return res.mb.body(pred);
      }
      else
      {
        return res.mb.body(0);
      }
    }();
    parent_node->SetAttribute("link", parent.name().c_str());

    auto succ = res.mb.successor(index);
    const auto & child = res.mb.body(succ);
    auto child_node = doc.NewElement("child");
    child_node->SetAttribute("link", child.name().c_str());

    node->InsertEndChild(parent_node);
    node->InsertEndChild(child_node);

    for(const auto & arc : res.mbg.nodeByName(parent.name())->arcs)
    {
      if(arc.joint.name() == joint.name())
      {
        set_origin_from_ptransform(node, arc.X);
      }
    }

    auto axis_node = doc.NewElement("axis");
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    switch(joint.type())
    {
      case Joint::Type::Rev:
        axis = joint.motionSubspace().col(0).head<3>();
        break;
      case Joint::Type::Prism:
        axis = joint.motionSubspace().col(0).tail<3>();
        break;
      case Joint::Type::Spherical:
        throw std::invalid_argument("URDF: Spherical is an unsupported joint type");
        break;
      case Joint::Type::Planar:
        axis = Eigen::Vector3d::UnitZ(); // Axis is not handled is RBDyn and defaults to Z
        break;
      case Joint::Type::Cylindrical:
        throw std::invalid_argument("URDF: Cylindrical is an unsupported joint type");
        break;
      case Joint::Type::Free:
        break;
      case Joint::Type::Fixed:
        break;
    }
    if(!axis.isZero())
    {
      set_vec3d(axis_node, "xyz", axis);
      node->InsertEndChild(axis_node);
    }

    if(has_limits(joint) && !(joint.type() == Joint::Type::Rev && is_continuous(joint)))
    {
      auto limit_node = doc.NewElement("limit");
      set_limit(limit_node, joint, "lower", res.limits.lower);
      set_limit(limit_node, joint, "upper", res.limits.upper);
      set_limit(limit_node, joint, "velocity", res.limits.velocity);
      set_limit(limit_node, joint, "effort", res.limits.torque);
      node->InsertEndChild(limit_node);
    }

    if(joint.isMimic())
    {
      auto mimic_node = doc.NewElement("mimic");
      mimic_node->SetAttribute("joint", joint.mimicName().c_str());
      set_double(mimic_node, "multiplier", joint.mimicMultiplier());
      set_double(mimic_node, "offset", joint.mimicOffset());
      node->InsertEndChild(mimic_node);
    }

    robot->InsertEndChild(node);
  }

  doc.InsertEndChild(robot);

  XMLPrinter printer;
  doc.Print(&printer);

  return std::string(printer.CStr());
}

} // namespace parsers
} // namespace rbd
