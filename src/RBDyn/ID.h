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

/**
	* Inverse Dynamics algorithm.
	*/
class RBDYN_DLLAPI InverseDynamics
{
public:
	InverseDynamics()
	{}
	/// @param mb MultiBody associated with this algorithm.
	InverseDynamics(const MultiBody& mb);

	/**
		* Compute the inverse dynamics.
		* @param mb MultiBody used has model.
		* @param mbc Use alphaD generalized acceleration vector, force, jointConfig,
		* jointVelocity, bodyPosW, parentToSon, bodyVelV, motionSubspace and gravity.
		* Fill bodyAccB and jointTorque.
		*/
	void inverseDynamics(const MultiBody& mb, MultiBodyConfig& mbc);
	/**
		* Compute the inverse dynamics with the inertia parameters.
		* @param mb MultiBody used has model.
		* @param mbc Use force, bodyPosW, parentToSon and motionSubspace.
		* Fill jointTorque.
		*/
	void inverseDynamicsNoInertia(const MultiBody& mb, MultiBodyConfig& mbc);

	// safe version for python binding

	/** safe version of @see inverseDynamics.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sInverseDynamics(const MultiBody& mb, MultiBodyConfig& mbc);
	/** safe version of @see inverseDynamicsNoInertia.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sInverseDynamicsNoInertia(const MultiBody& mb, MultiBodyConfig& mbc);

	/**
		* @brief Get the internal forces.
		* @return vector of forces transmitted from body λ(i) to body i across
		* joint i.
		*/
	const std::vector<sva::ForceVecd>& f() const;

private:
	/**
		* @brief Compute joint torques.
		* @param mb MultiBody used has model.
		* @param mbc Use force, bodyPosW, parentToSon and motionSubspace.
		* Fill jointTorque.
		*/
	void computeJointTorques(const MultiBody& mb, MultiBodyConfig& mbc);

private:
	/// @brief Internal forces.
	/// f_ is the vector of forces transmitted from body λ(i) to body i across
	/// joint i.
	std::vector<sva::ForceVecd> f_;
};

} // namespace rbd
