/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/parsers/urdf.h>
#include <RBDyn/parsers/yaml.h>

namespace rbdyn_parsers
{
const rbd::Geometry::Mesh & getMesh(const rbd::Geometry & geom)
{
  return boost::get<rbd::Geometry::Mesh>(geom.data);
}

const rbd::Geometry::Box & getBox(const rbd::Geometry & geom)
{
  return boost::get<rbd::Geometry::Box>(geom.data);
}

const rbd::Geometry::Cylinder & getCylinder(const rbd::Geometry & geom)
{
  return boost::get<rbd::Geometry::Cylinder>(geom.data);
}

const rbd::Geometry::Sphere & getSphere(const rbd::Geometry & geom)
{
  return boost::get<rbd::Geometry::Sphere>(geom.data);
}

const rbd::Geometry::Superellipsoid & getSuperellipsoid(const rbd::Geometry & geom)
{
  return boost::get<rbd::Geometry::Superellipsoid>(geom.data);
}

void setMesh(rbd::Geometry & geom, const rbd::Geometry::Mesh & data)
{
  geom.data = data;
}

void setBox(rbd::Geometry & geom, const rbd::Geometry::Box & data)
{
  geom.data = data;
}

void setCylinder(rbd::Geometry & geom, const rbd::Geometry::Cylinder & data)
{
  geom.data = data;
}

void setSphere(rbd::Geometry & geom, const rbd::Geometry::Sphere & data)
{
  geom.data = data;
}

void setSuperellipsoid(rbd::Geometry & geom, const rbd::Geometry::Superellipsoid & data)
{
  geom.data = data;
}
} // namespace rbdyn_parsers
