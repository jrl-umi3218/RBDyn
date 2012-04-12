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

// Eigen
#include <Eigen/Core>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{
class MultiBody;
class MultiBodyConfig;

/**
	* Forward Dynamics algorithm.
	*/
class ForwardDynamics
{
public:
	/// @param mb MultiBody associated with this algorithm.
	ForwardDynamics(const MultiBody& mb);

	/**
		* Compute the forward dynamics.
		* @param mb MultiBody used has model.
		* @param mbc Use parentToSon, motionSubspace jointVelocity, bodyVelB,
		* bodyPosW, force, gravity and jointTorque.
		* Fill alphaD generalized acceleration vector.
		*/
	void forwardDynamics(const MultiBody& mb, MultiBodyConfig& mbc);

	/**
		* Compute the inertia matrix H.
		* @param mb MultiBody used has model.
		* @param mbc Use parentToSon and motionSubspace.
		*/
	void computeH(const MultiBody& mb, MultiBodyConfig& mbc);

	/**
		* Compute the non linear effect vector (coriolis, gravity, external force).
		* @param mb MultiBody used has model.
		* @param mbc Use parentToSon, motionSubspace jointVelocity, bodyVelB,
		* bodyPosW, force and gravity.
		*/
	void computeC(const MultiBody& mb, MultiBodyConfig& mbc);


	/// @return The inertia matrix H.
	const Eigen::MatrixXd& H() const
	{
		return H_;
	}

	/// @return The non linear effect vector (coriolis, gravity, external force).
	const Eigen::VectorXd& C() const
	{
		return C_;
	}

	// safe version for python binding

	/** safe version of @see forwardDynamics.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sForwardDynamics(const MultiBody& mb, MultiBodyConfig& mbc);

	/** safe version of @see computeH.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeH(const MultiBody& mb, MultiBodyConfig& mbc);

	/** safe version of @see computeC.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeC(const MultiBody& mb, MultiBodyConfig& mbc);

private:
	Eigen::MatrixXd H_;
	Eigen::VectorXd C_;

	// H computation
	std::vector<sva::RBInertia> I_st_;
	std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> F_;

	// C computation
	std::vector<sva::MotionVec> acc_;
	std::vector<sva::ForceVec> f_;

	std::vector<int> dofPos_;
};

} // namespace rbd

