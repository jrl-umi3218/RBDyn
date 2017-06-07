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

// includes
// std
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/// Return the IMPhi matrix that compute I*m = IMPhi(m)*phi_i.
RBDYN_DLLAPI Eigen::Matrix<double, 6, 10> IMPhi(const sva::MotionVecd& mv);

/** Convert a RBInertiad into a phi vector.
 *  We define the inertial parameters of a body i as the 10d vector
 *  phi_i = [m, h, I] = [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Yzz]
 */

RBDYN_DLLAPI Eigen::Matrix<double, 10, 1> inertiaToVector(const sva::RBInertiad& rbi);


/// Convert a phi vector into a RBInertiad.
RBDYN_DLLAPI sva::RBInertiad vectorToInertia(const Eigen::Matrix<double, 10, 1>& vec);

/**
 * Safe version of @see vectorToInertia.
 * @throw std::out_of_range if the vector don't have 10 rows.
 */
RBDYN_DLLAPI sva::RBInertiad sVectorToInertia(const Eigen::VectorXd& vec);

/** Apply inertiaToVector to all MultiBody Body and concatenate it into one vector
 * Phi = [phi_0, ..., phi_N]
 */
RBDYN_DLLAPI Eigen::VectorXd multiBodyToInertialVector(const rbd::MultiBody& mb);

/**
 * IDIM stand for Inverse Dynamics Identification Model. It is used to
 * estimate the inertial parameters of a rigid body system.
 * This class allows to compute the Y matrix that linearizes the dynamics w.r.t.
 * the Phi vector and the torque: torque = Y*Phi.
 * The Y matrix is the concatenation of Y_{BI} and Y_{JI} matrix in the paper :
 * Study on Dynamics Identification of the Foot Viscoelasticity of a Humanoid Robot
 * Mikami, Yuya
 * Moulard, Thomas
 * Yoshida, Eiichi
 * Venture, Gentiane.
 */

class RBDYN_DLLAPI IDIM
{
public:
	IDIM()
	{}
	/// @param mb MultiBody associated with this algorithm.
	IDIM(const rbd::MultiBody& mb);

	/**
		* Compute the Y matrix.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB, bodyAccB, parentToSon and motionSubspace.
		* bodyAccB must been calculated with the gravity.
		*/
	void computeY(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

	/// Return the Y matrix.
	const Eigen::MatrixXd& Y() const
	{
		return Y_;
	}

	// safe version for python binding

	/** safe version of @see inverseDynamics.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeY(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

private:
	Eigen::MatrixXd Y_;
};

} // rbd
