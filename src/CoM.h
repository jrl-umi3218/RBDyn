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

// std
#include <vector>

// Eigen
#include <Eigen/Core>

// RBDyn
#include "Jacobian.h"

namespace rbd
{
class MultiBody;
class MultiBodyConfig;

/**
	* Compute the Center of Mass (CoM) position of a multibody.
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW.
	* @return CoM position in world frame.
	*/
Eigen::Vector3d computeCoM(const MultiBody& mb, const MultiBodyConfig& mbc);


/**
	* Compute the Center of Mass (CoM) velocity of a multibody.
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW and bodyVelB.
	* @return CoM velocity in world frame.
	*/
Eigen::Vector3d computeCoMVelocity(const MultiBody& mb, const MultiBodyConfig& mbc);


/**
	* Compute the Center of Mass (CoM) acceleration of a multibody.
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW, bodyVelW, bodyVelB, and bodyAccB.
	* @return CoM velocity in world frame.
	*/
Eigen::Vector3d computeCoMAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc);


/**
	* Compute the CoM jacobian with a simple but slow algorithm.
	*/
class CoMJacobianDummy
{
public:
	CoMJacobianDummy();

	/// @param mb MultiBody used has model.
	CoMJacobianDummy(const MultiBody& mb);
	/**
	 * @param mb MultiBody used has model.
	 * @param weight Per body weight.
	 */
	CoMJacobianDummy(const MultiBody& mb, std::vector<double> weight);

	~CoMJacobianDummy();

	/**
		* Compute the CoM jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @return CoM Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute the time derivative of the CoM jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
		* @return Time derivativo of the jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	// safe version for python binding

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc.
		*/
	const Eigen::MatrixXd& sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see jacobianDot.
		* @throw std::domain_error If mb don't match mbc.
		*/
	const Eigen::MatrixXd& sJacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

private:
	void init(const rbd::MultiBody& mb);

private:
	Eigen::MatrixXd jac_;
	Eigen::MatrixXd jacDot_;
	Eigen::MatrixXd jacFull_;

	std::vector<Jacobian> jacVec_;
	double totalMass_;
	std::vector<double> bodiesWeight_;
};


/**
	* Compute the CoM jacobian
	*/
class CoMJacobian
{
public:
	CoMJacobian();

	/// @param mb MultiBody used has model.
	CoMJacobian(const MultiBody& mb);
	/**
	 * @param mb MultiBody used has model.
	 * @param weight Per body weight.
	 */
	CoMJacobian(const MultiBody& mb, const std::vector<double>& weight);

	/**
		* Compute the CoM jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @return CoM Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute the time derivative of the CoM jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
		* @return Time derivativo of the jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	// safe version for python binding

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc.
		*/
	const Eigen::MatrixXd& sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see jacobianDot.
		* @throw std::domain_error If mb don't match mbc.
		*/
	const Eigen::MatrixXd& sJacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

private:
	void init(const rbd::MultiBody& mb, const std::vector<double>& weight);

private:
	Eigen::MatrixXd jac_;
	Eigen::MatrixXd jacDot_;

	std::vector<double> bodiesCoeff_;
	std::vector<sva::PTransformd> bodiesCoM_;
	std::vector<std::vector<int>> jointsSubBodies_;

	// jacobian, jacobianDot computation buffer
	std::vector<sva::PTransformd> bodiesCoMWorld_;
	std::vector<sva::MotionVecd> bodiesCoMVelB_;
};


// safe version for python binding

/**
	* Safe version.
	* @see computeCoM.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
Eigen::Vector3d sComputeCoM(const MultiBody& mb, const MultiBodyConfig& mbc);


/**
	* Safe version.
	* @see computeCoMVelocity.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
Eigen::Vector3d sComputeCoMVelocity(const MultiBody& mb, const MultiBodyConfig& mbc);


/**
	* Safe version.
	* @see computeCoMAcceleration.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
Eigen::Vector3d sComputeCoMAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc);


} // namespace rbd
