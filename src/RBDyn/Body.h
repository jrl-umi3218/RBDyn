/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// std
#include <string>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{

/** Body representation.
 * Hold body id, name and spatial rigid body inertia of one body.
 */
class Body
{
public:
  Body() {}

  /**
   * @param rbInertia Body spatial rigid body inertia.
   * @param name Body name, must be unique in a multibody.
   */
  Body(const sva::RBInertiad & rbInertia, std::string name) : inertia_(rbInertia), name_(name) {}

  /**
   * @param mass Body mass.
   * @param com Body center of mass.
   * @param inertia Body inertia matrix at body origin.
   * @param name Body name, must be unique in a multibody.
   */
  Body(double mass, const Eigen::Vector3d & com, const Eigen::Matrix3d & inertia, std::string name)
  : inertia_(mass, mass * com, inertia), name_(name)
  {
  }

  /// @return Body name.
  const std::string & name() const
  {
    return name_;
  }

  /// @return Body spatial rigid body inertia.
  const sva::RBInertiad & inertia() const
  {
    return inertia_;
  }

  bool operator==(const Body & b) const
  {
    return name_ == b.name_;
  }

  bool operator!=(const Body & b) const
  {
    return name_ != b.name_;
  }

private:
  sva::RBInertiad inertia_;
  std::string name_;
};

inline std::ostream & operator<<(std::ostream & out, const Body & b)
{
  out << "Body: " << b.name();
  return out;
}

} // namespace rbd
