// Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
//
// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

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
