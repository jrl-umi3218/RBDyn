/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// std
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include "Jacobian.h"

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

namespace ik
{

static constexpr int MAX_ITERATIONS = 50;
static constexpr double LAMBDA = 0.9;
static constexpr double THRESHOLD = 1e-8;
static constexpr double ALMOST_ZERO = 1e-8;

} // namespace ik

/**
 * Inverse Kinematics algorithm.
 */
class RBDYN_DLLAPI InverseKinematics
{
public:
  /// @param mb MultiBody associated with this algorithm.
  InverseKinematics(const MultiBody & mb, int ef_index);
  /**
   * Compute the inverse kinematics.
   * @param mb MultiBody used has model.
   * @param mbc Use q generalized position vector
   * @return bool if computation has converged
   * Fill q with new generalized position, update bodyPosW,
   * jointConfig and parentToSon. All computations are done
   * in-place : even if computation does not converge,
   * mbc will be modified.
   */
  bool inverseKinematics(const MultiBody & mb, MultiBodyConfig & mbc, const sva::PTransformd & ef_target);

  /** safe version of @see inverseKinematics.
   * @throw std::domain_error If mb doesn't match mbc.
   */
  bool sInverseKinematics(const MultiBody & mb, MultiBodyConfig & mbc, const sva::PTransformd & ef_target);
  /**
   * @brief Find q that minimizes the distance between ef and ef_target.
   * @return Bool if convergence has been reached
   */

  // @brief Maximum number of iterations
  int max_iterations_;
  // @brief Learning rate
  double lambda_;
  // @brief Stopping criterion
  double threshold_;
  // @brief Rounding threshold for the Jacobian
  double almost_zero_;

private:
  // @brief ef_index is the End Effector index used to build jacobian
  int ef_index_;
  Jacobian jac_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
};

} // namespace rbd
