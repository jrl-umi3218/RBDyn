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

// Eigen
#include <Eigen/Core>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
	* Forward Dynamics algorithm.
	*/
class RBDYN_DLLAPI ForwardDynamics
{
public:
	ForwardDynamics()
	{}
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
	void computeH(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute the non linear effect vector (coriolis, gravity, external force).
		* @param mb MultiBody used has model.
		* @param mbc Use parentToSon, motionSubspace jointVelocity, bodyVelB,
		* bodyPosW, force and gravity.
		*/
	void computeC(const MultiBody& mb, const MultiBodyConfig& mbc);


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

	/// @return Inertia of tho subtree rooted at body i.
	const std::vector<sva::RBInertiad>& inertiaSubTree() const
	{
		return I_st_;
	}

	// safe version for python binding

	/** safe version of @see forwardDynamics.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sForwardDynamics(const MultiBody& mb, MultiBodyConfig& mbc);

	/** safe version of @see computeH.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeH(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see computeC.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeC(const MultiBody& mb, const MultiBodyConfig& mbc);

private:
	Eigen::MatrixXd H_;
	Eigen::VectorXd C_;

	// H computation
	std::vector<sva::RBInertiad> I_st_;
	std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> F_;

	// C computation
	std::vector<sva::MotionVecd> acc_;
	std::vector<sva::ForceVecd> f_;

	// torque computation
	Eigen::VectorXd tmpFd_;

	std::vector<int> dofPos_;

	Eigen::LDLT<Eigen::MatrixXd> ldlt_;
};

} // namespace rbd

