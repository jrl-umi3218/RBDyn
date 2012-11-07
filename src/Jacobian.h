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
// RBDyn
#include "MultiBody.h"

namespace rbd
{
class MultiBodyConfig;

/**
	* Algorithm to compute the jacobian of a specified body.
	*/
class Jacobian
{
public:
	Jacobian();

	/**
		* Create a jacobian from the root body to the specified body.
		* @param mb Multibody where bodyId is in.
		* @param bodyId Specified body.
		* @param point Point in the body exprimed in body coordinate.
		* @throw std::out_of_range If bodyId don't exist.
		*/
	Jacobian(const MultiBody& mb, int bodyId,
		const Eigen::Vector3d& point=Eigen::Vector3d::Zero());

	/**
		* Compute the jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute the time derivative of the jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
		* @return Time derivativo of the jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	/**
		* Translate a jacobian at a given position.
		* @param jac Jacobian to translate.
		* @param mbc Use bodyPosW.
		* @param point Point to translate jacobian.
		* @param res Translated jacobian (must be allocated).
		*/
	void translateJacobian(const Eigen::MatrixXd& jac,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
		Eigen::MatrixXd& res);

	/**
		* Project the jacobian in the full robot parameters vector.
		* @param mb MuliBody used has model.
		* @param jac Jacobian to project.
		* @param res Projected Jacobian (must be allocated).
		*/
	void fullJacobian(const MultiBody& mb, const Eigen::MatrixXd& jac,
		Eigen::MatrixXd& res);

	/// @return MultiBody that correspond to the path between the root and
	/// the specified body.
	MultiBody subMultiBody(const MultiBody& mb) const;

	/// @return The joint path vector from the root to the specified body.
	const std::vector<int>& jointsPath() const
	{
		return jointsPath_;
	}

	/// @return The number of degree of freedom in the joint path
	int dof() const
	{
		return jac_.cols();
	}

	/// @return Static translation in the body exprimed in body coordinate.
	const Eigen::Vector3d& point() const
	{
		return point_.translation();
	}

	/// @param point Static translation in the body exprimed in body coordinate.
	void point(const Eigen::Vector3d& point)
	{
		point_ = sva::PTransform(point);
	}


	// safe version for python binding

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see subMultiBody.
		* @throw std::domain_error If mb don't match jointPath.
		*/
	MultiBody sSubMultiBody(const MultiBody& mb) const;

	/** safe version of @see jacobianDot.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sJacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	/** safe version of @see translateJacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath or res
		* size missmatch.
		*/
	void sTranslateJacobian(const Eigen::MatrixXd& jac,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
		Eigen::MatrixXd& res);

	/** safe version of @see fullJacobian.
		* @throw std::domain_error If mb don't match jointPath or res
		* size missmatch.
		*/
	void sFullJacobian(const MultiBody& mb, const Eigen::MatrixXd& jac,
		Eigen::MatrixXd& res);

private:
	std::vector<int> jointsPath_;
	sva::PTransform point_;

	Eigen::MatrixXd jac_;
	Eigen::MatrixXd jacDot_;
};

} // namespace rbd

