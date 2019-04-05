/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// includes
// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
	* Compute the forward acceleration of a MultiBody.
	* @param mb MultiBody used has model.
	* @param mbc Use alphaD generalized acceleration vector, jointVelocity,
	* parentToSon and bodyVelB.
	* Fill bodyAccB.
	* @param A_0 initial acceleration in world coordinate.
	*/
RBDYN_DLLAPI void forwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc,
	const sva::MotionVecd& A_0=sva::MotionVecd(Eigen::Vector6d::Zero()));

/**
	* Safe version.
	* @see forwardAcceleration.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
RBDYN_DLLAPI void sForwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc,
	const sva::MotionVecd& A_0=sva::MotionVecd(Eigen::Vector6d::Zero()));

} // namespace rbd
