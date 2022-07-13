/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// std
#include <vector>

// RBDyn
#include <rbdyn/config.hh>

#include "Joint.h"

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
 * Integrate a constant rotation acceleration.
 * @param qi Initial orientation.
 * @param wi Initial rotation speed.
 * @param wD Constant acceleration.
 * @param step Integration step.
 * @param relEps Stopping criterion for the underlying Magnus expansion, relatively to the norm of its first term.
 * @param absEps Absolute precision required on the underlying Magnus expansion.
 * @param breakOnWarning If true, computation is stopped right after the underlying Magnus expansion when it returns
 * a warning, and qi is returned, otherwise perform the full computation.
 */
RBDYN_DLLAPI std::pair<Eigen::Quaterniond, bool> SO3Integration(const Eigen::Quaterniond & qi,
                                                                const Eigen::Vector3d & wi,
                                                                const Eigen::Vector3d & wD,
                                                                double step,
                                                                double relEps = 1e-12,
                                                                double absEps = std::numeric_limits<double>::epsilon(),
                                                                bool breakOnWarning = false);

/**
 * Integrate joint configuration.
 * @param type Joint type.
 * @param alpha Joint velocity vector.
 * @param alphaD Joint acceleration vector.
 * @param step Integration step.
 * @param q Joint configuration vector.
 * @param prec Absolute precision used by numerical integrators.
 */
RBDYN_DLLAPI void jointIntegration(Joint::Type type,
                                   const std::vector<double> & alpha,
                                   const std::vector<double> & alphaD,
                                   double step,
                                   std::vector<double> & q,
                                   double prec = 1e-10);

/**
 * Use numerical or Euler methods (depending on joints) to compute the joint
 * configurations and velocities after a step with constant joint accelerations.
 * @param mb MultiBody used has model.
 * @param mbc Use alphaD, alpha and q. Fill alpha and q.
 * @param step Integration step.
 * @param prec Absolute precision used by numerical integrators.
 */
RBDYN_DLLAPI void integration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec = 1e-10);

/// safe version of @see integration.
RBDYN_DLLAPI void sIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec = 1e-10);

} // namespace rbd
