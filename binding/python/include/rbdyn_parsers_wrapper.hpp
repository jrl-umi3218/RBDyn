/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/parsers/urdf.h>
#include <RBDyn/parsers/yaml.h>

namespace rbdyn_parsers
{
const rbd::parsers::Geometry::Mesh & getMesh(const rbd::parsers::Geometry & geom)
{
  return boost::get<rbd::parsers::Geometry::Mesh>(geom.data);
}

const rbd::parsers::Geometry::Box & getBox(const rbd::parsers::Geometry & geom)
{
  return boost::get<rbd::parsers::Geometry::Box>(geom.data);
}

const rbd::parsers::Geometry::Cylinder & getCylinder(const rbd::parsers::Geometry & geom)
{
  return boost::get<rbd::parsers::Geometry::Cylinder>(geom.data);
}

const rbd::parsers::Geometry::Sphere & getSphere(const rbd::parsers::Geometry & geom)
{
  return boost::get<rbd::parsers::Geometry::Sphere>(geom.data);
}

const rbd::parsers::Geometry::Superellipsoid & getSuperellipsoid(const rbd::parsers::Geometry & geom)
{
  return boost::get<rbd::parsers::Geometry::Superellipsoid>(geom.data);
}

void setMesh(rbd::parsers::Geometry & geom, const rbd::parsers::Geometry::Mesh & data)
{
  geom.data = data;
}

void setBox(rbd::parsers::Geometry & geom, const rbd::parsers::Geometry::Box & data)
{
  geom.data = data;
}

void setCylinder(rbd::parsers::Geometry & geom, const rbd::parsers::Geometry::Cylinder & data)
{
  geom.data = data;
}

void setSphere(rbd::parsers::Geometry & geom, const rbd::parsers::Geometry::Sphere & data)
{
  geom.data = data;
}

void setSuperellipsoid(rbd::parsers::Geometry & geom, const rbd::parsers::Geometry::Superellipsoid & data)
{
  geom.data = data;
}
} // namespace rbdyn_parsers
