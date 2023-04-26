#include <RBDyn/parsers/yaml.h>

#include <yaml-cpp/yaml.h>

namespace rbd
{
namespace parsers
{

std::string to_yaml(const ParserResult & res)
{
  using namespace YAML;

  Emitter doc;

  doc << BeginMap;
  doc << Key << "robot" << Value << BeginMap;
  doc << Key << "name" << Value << res.name;
  doc << Key << "anglesInDegrees" << Value << false;

  auto set_vec3d = [&](const char * name, Eigen::Ref<const Eigen::Vector3d> xyz)
  {
    doc << Value << name << Key << Flow << BeginSeq;
    doc << xyz.x() << xyz.y() << xyz.z();
    doc << EndSeq;
  };

  auto set_origin_from_ptransform = [&](const sva::PTransformd & X)
  {
    const auto & xyz = X.translation();
    const auto rpy = X.rotation().transpose().eulerAngles(0, 1, 2);
    if(!xyz.isZero() || !rpy.isZero())
    {
      doc << Key << "frame" << BeginMap;
      if(!xyz.isZero())
      {
        set_vec3d("xyz", xyz);
      }
      if(!rpy.isZero())
      {
        set_vec3d("rpy", rpy);
      }
      doc << EndMap;
    }
  };

  // Links
  doc << Key << "links" << Value << BeginSeq;
  for(const auto & link : res.mb.bodies())
  {
    doc << BeginMap;
    doc << Key << "name" << Value << link.name();

    // Inertial
    const auto has_mass = link.inertia().mass() > 0.;
    const auto has_momentum = !link.inertia().momentum().isZero();
    const auto has_inertia = !link.inertia().inertia().isZero();
    if(has_mass || has_momentum || has_inertia)
    {
      doc << Key << "inertial" << BeginMap;

      if(link.inertia().mass() > 0.)
      {
        doc << Key << "mass" << Value << link.inertia().mass();
      }

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
        doc << Key << "frame" << BeginMap;
        set_vec3d("xyz", com);
        doc << EndMap;
      }

      if(!link.inertia().inertia().isZero())
      {
        //! Transform the inertia before writing it so that it can be later read using the 'transformInertia' parameter
        //! to false (default value)
        const auto inertia = sva::inertiaToOrigin(link.inertia().inertia(), -link.inertia().mass(), com,
                                                  Eigen::Matrix3d::Identity().eval());

        doc << Key << "inertia" << BeginMap;
        doc << Key << "Ixx" << Value << inertia(0, 0);
        doc << Key << "Iyy" << Value << inertia(1, 1);
        doc << Key << "Izz" << Value << inertia(2, 2);
        doc << Key << "Iyz" << Value << inertia(1, 2);
        doc << Key << "Ixz" << Value << inertia(0, 2);
        doc << Key << "Ixy" << Value << inertia(0, 1);
        doc << EndMap;
      }

      doc << EndMap;
    }

    auto generate_visual = [&](const char * type, const std::map<std::string, std::vector<Visual>> & visuals)
    {
      auto visuals_it = visuals.find(link.name());
      if(visuals_it == visuals.end())
      {
        return;
      }
      doc << Key << type << Value << BeginSeq;
      for(size_t i = 0; i < visuals_it->second.size(); ++i)
      {
        const auto & visual = visuals_it->second[i];
        if(visual.geometry.type == Geometry::Type::UNKNOWN)
        {
          continue;
        }

        doc << BeginMap;

        set_origin_from_ptransform(visual.origin);

        const auto & material = visual.material;
        if(material.type != Material::Type::NONE)
        {
          doc << Key << "material" << Value << BeginMap;
          doc << Key << "name" << Value << ("material_" + link.name() + "_" + std::to_string(i));
          if(material.type == Material::Type::COLOR)
          {
            const auto & c = boost::get<Material::Color>(material.data);
            doc << Key << "color" << Value << BeginMap;
            doc << Key << "rgba" << Value << Flow << BeginSeq << c.r << c.g << c.b << c.a << EndSeq;
            doc << EndMap;
          }
          else if(material.type == Material::Type::TEXTURE)
          {
            const auto & texture = boost::get<Material::Texture>(material.data);
            doc << Key << "texture" << Value << BeginMap;
            doc << Key << "filename" << Value << prefix_path(texture.filename);
            doc << EndMap;
          }
          doc << EndMap;
        }

        doc << Key << "geometry" << Value << BeginMap;

        switch(visual.geometry.type)
        {
          case Geometry::Type::BOX:
          {
            doc << Key << "box" << BeginMap;
            const auto box = boost::get<Geometry::Box>(visual.geometry.data);
            set_vec3d("size", box.size);
            doc << EndMap;
          }
          break;
          case Geometry::Type::CYLINDER:
          {
            doc << Key << "cylinder" << BeginMap;
            const auto cylinder = boost::get<Geometry::Cylinder>(visual.geometry.data);
            doc << Key << "radius" << Value << cylinder.radius;
            doc << Key << "length" << Value << cylinder.length;
            doc << EndMap;
          }
          break;
          case Geometry::Type::MESH:
          {
            doc << Key << "mesh" << BeginMap;
            const auto mesh = boost::get<Geometry::Mesh>(visual.geometry.data);
            doc << Key << "filename" << Value << prefix_path(mesh.filename);
            doc << Key << "scale" << BeginSeq << mesh.scaleV.x() << mesh.scaleV.y() << mesh.scaleV.z() << EndSeq;
            doc << EndMap;
          }
          break;
          case Geometry::Type::SPHERE:
          {
            doc << Key << "sphere" << BeginMap;
            const auto sphere = boost::get<Geometry::Sphere>(visual.geometry.data);
            doc << Key << "radius" << Value << sphere.radius;
            doc << EndMap;
          }
          break;
          case Geometry::Type::SUPERELLIPSOID:
          {
            doc << Key << "superellipsoid" << BeginMap;
            const auto superellipsoid = boost::get<Geometry::Superellipsoid>(visual.geometry.data);
            set_vec3d("size", superellipsoid.size);
            doc << Key << "epsilon1" << Value << superellipsoid.epsilon1;
            doc << Key << "epsilon2" << Value << superellipsoid.epsilon2;
            doc << EndMap;
          }
          break;
          case Geometry::Type::UNKNOWN:
            break;
        }

        doc << EndMap << EndMap;
      }

      doc << EndSeq;
    };

    generate_visual("visual", res.visual);
    generate_visual("collision", res.collision);

    doc << EndMap;
  }
  doc << EndSeq;

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

  auto set_vector = [&](const char * name, const std::vector<double> & v)
  { doc << Key << name << Value << Flow << BeginSeq << v << EndSeq; };

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

  auto set_limit =
      [&](const Joint & joint, const char * name, const std::map<std::string, std::vector<double>> & limits)
  {
    auto it = limits.find(joint.name());
    if(it != limits.end())
    {
      if(it->second.size() == 1)
      {
        doc << Key << name << Value << it->second[0];
      }
      else
      {
        set_vector(name, it->second);
      }
    }
  };

  // Joints
  doc << Key << "joints" << Value << BeginSeq;
  for(const auto & joint : res.mb.joints())
  {
    // Skip the root joint
    if(res.mb.jointIndexByName(joint.name()) == 0)
    {
      continue;
    }

    doc << BeginMap;
    doc << Key << "name" << Value << joint.name();

    switch(joint.type())
    {
      case Joint::Type::Rev:
        if(is_continuous(joint))
        {
          doc << Key << "type" << Value << "continuous";
        }
        else
        {
          doc << Key << "type" << Value << "revolute";
        }
        break;
      case Joint::Type::Prism:
        doc << Key << "type" << Value << "prismatic";
        break;
      case Joint::Type::Spherical:
        throw std::invalid_argument("URDF: Spherical is an unsupported joint type");
        break;
      case Joint::Type::Planar:
        doc << Key << "type" << Value << "planar";
        break;
      case Joint::Type::Cylindrical:
        throw std::invalid_argument("URDF: Cylindrical is an unsupported joint type");
        break;
      case Joint::Type::Free:
        doc << Key << "type" << Value << "floating";
        break;
      case Joint::Type::Fixed:
        doc << Key << "type" << Value << "fixed";
        break;
    }

    auto index = res.mb.jointIndexByName(joint.name());
    const auto pred = res.mb.predecessor(index);
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
    doc << Key << "parent" << Value << parent.name();

    auto succ = res.mb.successor(index);
    const auto & child = res.mb.body(succ);
    doc << Key << "child" << Value << child.name();

    for(const auto & arc : res.mbg.nodeByName(parent.name())->arcs)
    {
      if(arc.joint.name() == joint.name())
      {
        set_origin_from_ptransform(arc.X);
      }
    }

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
      set_vec3d("axis", axis);
    }

    if(has_limits(joint) && !(joint.type() == Joint::Type::Rev && is_continuous(joint)))
    {
      doc << Key << "limits" << Value << BeginMap;
      set_limit(joint, "lower", res.limits.lower);
      set_limit(joint, "upper", res.limits.upper);
      set_limit(joint, "velocity", res.limits.velocity);
      set_limit(joint, "effort", res.limits.torque);
      doc << EndMap;
    }

    doc << EndMap;
  }
  doc << EndSeq; // Joints

  doc << EndMap; // Robot
  doc << EndMap; // Doc

  return std::string(doc.c_str());
}

} // namespace parsers
} // namespace rbd
