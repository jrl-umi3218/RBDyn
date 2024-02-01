/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/parsers/common.h>

constexpr double TOL = 1e-6;

inline rbd::parsers::ParserResult createRobot()
{
  auto create_ptransform = [](double x, double y, double z, double rx, double ry, double rz)
  {
    Eigen::Quaterniond q = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());

    return sva::PTransformd(q.inverse(), Eigen::Vector3d(x, y, z));
  };

  rbd::parsers::ParserResult res;

  Eigen::Matrix3d I0, I1, I2, I3, I4;

  I0 << 0.1, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.001;

  I1 << 1.35, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.251;

  I2 << 0.6, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.501;

  I3 << 0.475, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.376;

  I4 << 0.1, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.251;
  Eigen::Affine3d T0, T1;
  T0.matrix() << 1, 0, 0, .1, 0, 1, 0, .2, 0, 0, 1, .3, 0, 0, 0, 1;
  T1 = Eigen::Affine3d::Identity();

  rbd::Body b0(1., Eigen::Vector3d::Zero(), I0, "b0");
  rbd::Body b1(5., Eigen::Vector3d(0., 0.5, 0.), I1, "b1");
  rbd::Body b2(2., Eigen::Vector3d(0., 0.5, 0.), I2, "b2");
  rbd::Body b3(1.5, Eigen::Vector3d(0., 0.5, 0.), I3, "b3");
  rbd::Body b4(1., Eigen::Vector3d(0.5, 0., 0.), I4, "b4");

  res.mbg.addBody(b0);
  res.mbg.addBody(b1);
  res.mbg.addBody(b2);
  res.mbg.addBody(b3);
  res.mbg.addBody(b4);

  rbd::Joint j0(rbd::Joint::RevX, true, "j0");
  rbd::Joint j1(rbd::Joint::RevY, true, "j1");
  rbd::Joint j2(rbd::Joint::RevZ, true, "j2");
  rbd::Joint j3(rbd::Joint::RevX, true, "j3");

  res.mbg.addJoint(j0);
  res.mbg.addJoint(j1);
  res.mbg.addJoint(j2);
  res.mbg.addJoint(j3);

  sva::PTransformd to(Eigen::Vector3d(0., 1., 0.));
  sva::PTransformd from(sva::PTransformd::Identity());

  res.mbg.linkBodies("b0", to, "b1", from, "j0");
  res.mbg.linkBodies("b1", to, "b2", from, "j1");
  res.mbg.linkBodies("b2", to, "b3", from, "j2");
  res.mbg.linkBodies("b1", sva::PTransformd(sva::RotX(1.), Eigen::Vector3d(1., 0., 0.)), "b4", from, "j3");

  res.limits.lower = {{"j0", {-1.}}, {"j1", {-1.}}, {"j2", {-1.}}};
  res.limits.upper = {{"j0", {1.}}, {"j1", {1.}}, {"j2", {1.}}};
  res.limits.velocity = {{"j0", {10.}}, {"j1", {10.}}, {"j2", {10.}}};
  res.limits.torque = {{"j0", {50.}}, {"j1", {50.}}, {"j2", {50.}}};

  // b0 visuals
  {
    rbd::parsers::Visual v1, v2;
    rbd::parsers::Geometry::Mesh mesh;
    v1.origin = create_ptransform(0.1, 0.2, 0.3, 0, 0, 0);
    mesh.filename = "file://test_mesh1.dae";
    mesh.scaleV = Eigen::Vector3d::Ones();
    v1.geometry.type = rbd::parsers::Geometry::Type::MESH;
    v1.geometry.data = mesh;

    v2.origin = create_ptransform(0, 0, 0, 0, 0, 0);
    mesh.filename = "file://test_mesh2.dae";
    mesh.scaleV = Eigen::Vector3d(0.1, -0.1, 0.1);
    v2.geometry.type = rbd::parsers::Geometry::Type::MESH;
    v2.geometry.data = mesh;

    res.visual["b0"] = {v1, v2};
  }

  // b1 visual and collision
  {
    rbd::parsers::Visual v1, v2;
    rbd::parsers::Geometry::Box box;
    v1.origin = create_ptransform(0.4, 0.5, 0.6, 1, 0, 0);
    box.size << 1, 2, 3;
    v1.geometry.type = rbd::parsers::Geometry::Type::BOX;
    v1.geometry.data = box;

    v2.origin = create_ptransform(0.4, 0.5, 0.6, 0, 1, 0);
    box.size << 1, 2, 3;
    v2.geometry.type = rbd::parsers::Geometry::Type::BOX;
    v2.geometry.data = box;

    res.visual["b1"] = {v1};
    res.collision["b1"] = {v2};
  }

  // b2 visual
  {
    rbd::parsers::Visual v1;
    rbd::parsers::Geometry::Cylinder cylinder;
    v1.origin = create_ptransform(0.4, 0.5, 0.6, 0, 0, 1);
    cylinder.radius = 1;
    cylinder.length = 2;
    v1.geometry.type = rbd::parsers::Geometry::Type::CYLINDER;
    v1.geometry.data = cylinder;

    res.visual["b2"] = {v1};
  }

  // b3 visual
  {
    rbd::parsers::Visual v1;
    rbd::parsers::Geometry::Sphere sphere;
    v1.origin = create_ptransform(0.4, 0.5, 0.6, 1, 0, 0);
    sphere.radius = 2;
    v1.geometry.type = rbd::parsers::Geometry::Type::SPHERE;
    v1.geometry.data = sphere;
    v1.material.type = rbd::parsers::Material::Type::COLOR;
    v1.material.data = rbd::parsers::Material::Color{1.0, 0.0, 0.0, 1.0};

    res.visual["b3"] = {v1};
  }

  // b4 visual
  {
    rbd::parsers::Visual v1;
    rbd::parsers::Geometry::Superellipsoid superellipsoid;
    v1.origin = create_ptransform(0.4, 0.5, 0.6, 0, 1, 0);
    superellipsoid.size << 0.1, 0.2, 0.3;
    superellipsoid.epsilon1 = 0.5;
    superellipsoid.epsilon2 = 1;
    v1.geometry.type = rbd::parsers::Geometry::Type::SUPERELLIPSOID;
    v1.geometry.data = superellipsoid;
    v1.material.type = rbd::parsers::Material::Type::TEXTURE;
    v1.material.data = rbd::parsers::Material::Texture{"file:///some/texture.png"};

    res.visual["b4"] = {v1};
  }

  res.mb = res.mbg.makeMultiBody("b0", true);
  res.mbc = rbd::MultiBodyConfig(res.mb);
  res.mbc.zero(res.mb);

  return res;
}

namespace rbd
{

namespace parsers
{

inline bool operator==(const Geometry::Mesh & m1, const Geometry::Mesh & m2)
{
  return m1.scaleV == m2.scaleV && m1.filename == m2.filename;
}

inline bool operator==(const Geometry::Box & b1, const Geometry::Box & b2)
{
  return b1.size == b2.size;
}

inline bool operator==(const Geometry::Sphere & b1, const Geometry::Sphere & b2)
{
  return b1.radius == b2.radius;
}

inline bool operator==(const Geometry::Cylinder & b1, const Geometry::Cylinder & b2)
{
  return b1.radius == b2.radius && b1.length == b2.length;
}

inline bool operator==(const Geometry & g1, const Geometry & g2)
{
  return g1.type == g2.type && g1.data == g2.data;
}

inline bool operator==(const Material::Color & c1, const Material::Color & c2)
{
  return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b && c1.a == c2.a;
}

inline bool operator==(const Material::Texture & t1, const Material::Texture & t2)
{
  return t1.filename == t2.filename;
}

inline bool operator==(const Material & m1, const Material & m2)
{
  return m1.type == m2.type && (m1.type == Material::Type::NONE || m1.data == m2.data);
}

inline bool operator==(const Visual & v1, const Visual & v2)
{
  return v1.name == v2.name && v1.origin.rotation().isApprox(v2.origin.rotation(), TOL)
         && v1.origin.translation().isApprox(v2.origin.translation(), TOL) && v1.geometry == v2.geometry
         && v1.material == v2.material;
}

inline bool operator==(const Geometry::Superellipsoid & se1, const Geometry::Superellipsoid & se2)
{
  return (se1.size == se2.size && se1.epsilon1 == se2.epsilon1 && se1.epsilon2 == se2.epsilon2);
}

} // namespace parsers

} // namespace rbd

const std::string XYZSarmUrdf(
    R"(<robot name="XYZSarm">
    <link name="b0">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="0. 0. 0." xyz=".1 .2 .3"/>
        <geometry>
          <mesh filename="file://test_mesh1.dae"/>
        </geometry>
      </visual>
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="file://test_mesh2.dae" scale="0.1 -0.1 0.1" />
        </geometry>
      </visual>
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </visual>
    </link>
    <link name="b1">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="5." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="1 0 0" xyz=".4 .5 .6"/>
        <geometry>
          <box size="1 2 3"/>
        </geometry>
      </visual>
      <collision>
        <origin rpy="0 1 0" xyz=".4 .5 .6"/>
        <geometry>
          <box size="1 2 3"/>
        </geometry>
      </collision>
    </link>
    <link name="b2">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="2." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="0 0 1" xyz=".4 .5 .6"/>
        <geometry>
          <cylinder radius="1" length="2"/>
        </geometry>
      </visual>
    </link>
    <link name="b3">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="1.5" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="1 0 0" xyz=".4 .5 .6"/>
        <geometry>
          <sphere radius="2"/>
        </geometry>
        <material name="Red">
          <color rgba="1.0 0.0 0.0 1.0" />
        </material>
      </visual>
    </link>
    <link name="b4">
      <inertial>
        <origin rpy="0 0 0" xyz="0.5 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="0 1 0" xyz=".4 .5 .6"/>
        <geometry>
          <superellipsoid size="0.1 0.2 0.3" epsilon1="0.5" epsilon2="1"/>
        </geometry>
        <material name="Texture">
          <texture filename="file:///some/texture.png" />
        </material>
      </visual>
    </link>
    <joint name="j0" type="revolute">
      <parent link="b0" />
      <child link="b1" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="1 0 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j1" type="revolute">
      <parent link="b1" />
      <child link="b2" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j2" type="revolute">
      <parent link="b2" />
      <child link="b3" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 0 1" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j3" type="continuous">
      <parent link="b1" />
      <child link="b4" />
      <origin rpy="1. 0 0" xyz="1 0 0" />
      <axis xyz="1 0 0" />
    </joint>
  </robot>
)");

const std::string XYZSarmYaml(
    R"(robot:
  name: XYZSarm
  anglesInDegrees: false
  links:
    - name: b0
      inertial:
        mass: 1
        frame:
          xyz: [0, 0, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.1, 0.2, 0.3]
            rpy: [0, 0, 0]
          geometry:
            mesh:
              filename: file://test_mesh1.dae
        - frame:
            xyz: [0, 0, 0]
            rpy: [0, 0, 0]
          geometry:
            mesh:
              filename: file://test_mesh2.dae
              scale: [0.1, -0.1, 0.1]
        - frame:
            xyz: [0, 0, 0]
            rpy: [0, 0, 0]
    - name: b1
      inertial:
        mass: 5
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.4, 0.5, 0.6]
            rpy: [1, 0, 0]
          geometry:
            box:
              size: [1, 2, 3]
      collision:
        - frame:
            xyz: [0.4, 0.5, 0.6]
            rpy: [0, 1, 0]
          geometry:
            box:
              size: [1, 2, 3]
    - name: b2
      inertial:
        mass: 2
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.4, 0.5, 0.6]
            rpy: [0, 0, 1]
          geometry:
            cylinder:
              radius: 1
              length: 2
    - name: b3
      inertial:
        mass: 1.5
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.4, 0.5, 0.6]
            rpy: [1, 0, 0]
          geometry:
            sphere:
              radius: 2
          material:
            name: Red
            color:
              rgba: [1.0, 0.0, 0.0, 1.0]
    - name: b4
      inertial:
        mass: 1
        frame:
          xyz: [0.5, 0, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.4, 0.5, 0.6]
            rpy: [0, 1, 0]
          geometry:
            superellipsoid:
              size: [0.1, 0.2, 0.3]
              epsilon1: 0.5
              epsilon2: 1
          material:
            name: Texture
            texture:
              filename: file:///some/texture.png
  joints:
    - name: j0
      parent: b0
      child: b1
      type: revolute
      axis: [1, 0, 0]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j1
      parent: b1
      child: b2
      type: revolute
      axis: [0, 1, 0]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j2
      parent: b2
      child: b3
      type: revolute
      axis: [0, 0, 1]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j3
      parent: b1
      child: b4
      type: continuous
      axis: [1, 0, 0]
      frame:
        xyz: [1, 0, 0]
        rpy: [1, 0, 0]
        anglesInDegrees: false)");
