/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// std
#include <vector>

// SpaceVecAlg
#include <rbdyn/config.hh>

#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
 * Inverse Dynamics algorithm.
 */
class RBDYN_DLLAPI InverseDynamics
{
public:
  InverseDynamics() {}
  /// @param mb MultiBody associated with this algorithm.
  InverseDynamics(const MultiBody & mb);

  /**
   * Compute the inverse dynamics.
   * @param mb MultiBody used has model.
   * @param mbc Use alphaD generalized acceleration vector, force, jointConfig,
   * jointVelocity, bodyPosW, parentToSon, bodyVelV, motionSubspace and gravity.
   * Fill bodyAccB and jointTorque.
   */
  void inverseDynamics(const MultiBody & mb, MultiBodyConfig & mbc);
  /**
   * Compute the inverse dynamics with the inertia parameters.
   * @param mb MultiBody used has model.
   * @param mbc Use force, bodyPosW, parentToSon and motionSubspace.
   * Fill jointTorque.
   */
  void inverseDynamicsNoInertia(const MultiBody & mb, MultiBodyConfig & mbc);

  // safe version for python binding

  /** safe version of @see inverseDynamics.
   * @throw std::domain_error If mb don't match mbc.
   */
  void sInverseDynamics(const MultiBody & mb, MultiBodyConfig & mbc);
  /** safe version of @see inverseDynamicsNoInertia.
   * @throw std::domain_error If mb don't match mbc.
   */
  void sInverseDynamicsNoInertia(const MultiBody & mb, MultiBodyConfig & mbc);

  /**
   * @brief Get the internal forces.
   * @return vector of forces transmitted from body λ(i) to body i across
   * joint i.
   */
  const std::vector<sva::ForceVecd> & f() const;

private:
  /**
   * @brief Compute joint torques.
   * @param mb MultiBody used has model.
   * @param mbc Use force, bodyPosW, parentToSon and motionSubspace.
   * Fill jointTorque.
   */
  void computeJointTorques(const MultiBody & mb, MultiBodyConfig & mbc);

private:
  /// @brief Internal forces.
  /// f_ is the vector of forces transmitted from body λ(i) to body i across
  /// joint i.
  std::vector<sva::ForceVecd> f_;
};

} // namespace rbd
