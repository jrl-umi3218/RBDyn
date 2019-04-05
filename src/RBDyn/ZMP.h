/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

// include
// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

namespace rbd
{
struct MultiBodyConfig;
/**
	 * Compute the ZMP in the world frame
	 * as in Kajita's book on humanoid robots chap. 3 p.38
	 * @param mbc Use gravity
	 * @param com CoM position in world frame
	 * @oaram comA CoM acceleration in world frame, this must be computed without
	 *	the gravity term
	 * @param altitude Double representing the surface's altitude in world frame
	 * @return ZMP
	 */
RBDYN_DLLAPI Eigen::Vector3d computeCentroidalZMP(MultiBodyConfig& mbc, Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude);

/**
	 * @brief Compute the ZMP in the world frame considering that gravity is taken
	 * into account in the CoM acceleration
	 * @param com CoM position in world frame
	 * @param comA CoM acceleration in world frame, this must be computed with
	 *	the gravity term
	 * @param altitude Double representing the surface's altitude in world frame
	 * @return ZMP
	 */
Eigen::Vector3d computeCentroidalZMPNoGravity(Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude);

/**
	 * @brief Compute the ZMP considering the external wrench applied on the robot
	 * the gravity should also be already in the CoM acceleration
	 * @param mbc used for the gravity
	 * @param com CoM position in world frame
	 * @param comA CoM acceleration in world frame, this must be computed with
	 *	the gravity term
	 * @param altitude Double representing the surface's altitude in world frame
	 * @param wr_external External wrench acting on the CoM
	 * @param mass Mass of the robot
	 * @return ZMP
	 */
Eigen::Vector3d computeCentroidalZMPComplete(MultiBodyConfig& mbc, Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude, sva::ForceVecd wr_external, double mass);

}
