/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/MultiBodyConfig.h>

namespace rbd
{

/**
 * Computation of the feedforward friction vector according to the M0 model described
 * by Bittencourt and Gunnarsson in "Static Friction in a Robot Joint - 
 * Modeling and Identification of Load and Temperature Effects", 2012.
 */
class RBDYN_DLLAPI Friction
{
public:
  /** Initialize the required structures
   * @param mb Multibody system
   */
  Friction(const MultiBody & mb);

  /**
   * Compute the feedforward friction vector.
   * @param mb  MultiBody has model.
   * @param mbc MultiBodyConfig has alpha.
   */
  const Eigen::VectorXd & friction(const MultiBody & mb, const MultiBodyConfig & mbc);
  
private:
  Eigen::VectorXd Fr_;

  // friction computation
  std::vector<int> dofPos_;
};
 
}
