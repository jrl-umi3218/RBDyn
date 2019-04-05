/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// std
#include <vector>

// RBDyn
#include "Joint.h"

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
	* Integrate joint configuration.
	* @param type Joint type.
	* @param alpha Joint velocity vector.
	* @param alphaD Joint acceleration vector.
	* @param step Integration step.
	* @param q Joint configuration vector.
	*/
RBDYN_DLLAPI void eulerJointIntegration(Joint::Type type, const std::vector<double>& alpha,
	const std::vector<double>& alphaD,  double step, std::vector<double>& q);

/**
	* Use the euler method to integrate.
	* @param mb MultiBody used has model.
	* @param mbc Use alphaD, alpha and q. Fill alpha and q.
	* @param step Integration step.
	*/
RBDYN_DLLAPI void eulerIntegration(const MultiBody& mb, MultiBodyConfig& mbc, double step);

/// safe version of @see eulerIntegration.
RBDYN_DLLAPI void sEulerIntegration(const MultiBody& mb, MultiBodyConfig& mbc, double step);

} // namepsace rbd
