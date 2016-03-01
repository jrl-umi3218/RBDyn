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

#include "Jacobian.h"

namespace rbd
{
class MultiBody;
class MultiBodyConfig;

/**
	* Inverse Kinematics algorithm.
	*/
class InverseKinematics
{
public:
	InverseKinematics()
	{}
	/// @param mb MultiBody associated with this algorithm.
	InverseKinematics(const MultiBody& mb, int ef_index);
	/**
	* Compute the inverse kinematics.
	* @param mb MultiBody used has model.
	* @param mbc Use q generalized position vector
	* @return bool if computation has converged
	* Fill q with new generalized position
	*/
	bool inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
			       const sva::PTransformd& ef_target);

	/** safe version of @see inverseKinematics.
	* @throw std::domain_error If mb doesn't match mbc.
	*/
	bool sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
				const sva::PTransformd& ef_target);
	/**
	* @brief Find q that minimizes the distance between ef and ef_target.
	* @return Bool if convergence has been reached
	*/

private:
	// @brief ef_index is the End Effector index used to build jacobian
	int ef_index_;
	Jacobian jac_;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
};

} // namespace rbd
