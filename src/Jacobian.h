// Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
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
// RBDyn
#include "MultiBody.h"

#include <rbdyn/config.hh>

namespace rbd
{
struct MultiBodyConfig;

/**
	* Algorithm to compute the jacobian of a specified body.
	*/
class RBDYN_DLLAPI Jacobian
{
public:
	Jacobian();

	/**
		* Create a jacobian from the root body to the specified body.
		* @param mb Multibody where bodyId is in.
		* @param bodyName Specified body.
		* @param point Point in the body exprimed in body coordinate.
		* @throw std::out_of_range If bodyId don't exist.
		*/
	Jacobian(const MultiBody& mb, const std::string& bodyName,
		const Eigen::Vector3d& point=Eigen::Vector3d::Zero());

	/**
		* Compute the jacobian at the point/frame specified by X_0_p.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @param X_0_p Jacobian point/frame
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobian(const MultiBody& mb, const MultiBodyConfig& mbc,
			const sva::PTransformd& X_0_p);

	/**
		* Compute the jacobian in world frame.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute the jacobian in body coordinate frame.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& bodyJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/**
		* Compute a vector jacobian in world coordinate frame.
		* This function only fill the translation component of the jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @param vector Vector from then jacobian point (body coordinate).
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& vectorJacobian(const MultiBody& mb,
																					const MultiBodyConfig& mbc,
																					const Eigen::Vector3d& vector);

	/**
		* Compute a vector jacobian in body coordinate frame.
		* This function only fill the translation component of the jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW and motionSubspace.
		* @param vector Vector from then jacobian point (body coordinate).
		* @return Jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& vectorBodyJacobian(const MultiBody& mb,
																					const MultiBodyConfig& mbc,
																					const Eigen::Vector3d& vector);

	/**
		* Compute the time derivative of the jacobian.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
		* @return Time derivativo of the jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& jacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	/**
		* Compute the time derivative of the jacobian in body frame.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, bodyVelW, and motionSubspace.
		* @return Time derivativo of the jacobian of mb with mbc configuration.
		*/
	const Eigen::MatrixXd& bodyJacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	/**
		* Compute the end body point velocity at the point/frame specified by X_b_p.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB.
		* @param X_b_p velocity point/frame.
		* @return End body point velocity in world coordinate.
		*/
	sva::MotionVecd velocity(const MultiBody& mb,
		const MultiBodyConfig& mbc, const sva::PTransformd& X_b_p) const;

	/**
		* Compute the end body point velocity in world coordinate (J·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB.
		* @return End body point velocity in world coordinate.
		*/
	sva::MotionVecd velocity(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/**
		* Compute the end body point velocity in body coordinate (JBody·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB.
		* @return End body point velocity in body coordinate.
		*/
	sva::MotionVecd bodyVelocity(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/**
		* Compute the end body point normal acceleration in world coordinate (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB, jointVelocity, parentToSon.
		* @param X_b_p normal acceleration point/frame.
		* @param V_b_p X_b_p velocity.
		* @return End body point normal acceleration in world coordinate.
		*/
	sva::MotionVecd normalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const sva::PTransformd& X_b_p, const sva::MotionVecd& V_b_p) const;

	/**
		* Compute the end body point normal acceleration in world coordinate (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelW, bodyVelB, jointVelocity, parentToSon.
		* @return End body point normal acceleration in world coordinate.
		*/
	sva::MotionVecd normalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/**
		* Compute the end body point normal acceleration in body coordinate (JDotBody·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB, jointVelocity, parentToSon.
		* @return End body point normal acceleration in body coordinate.
		*/
	sva::MotionVecd bodyNormalAcceleration(const MultiBody& mb,
		const MultiBodyConfig& mbc) const;

	/**
		* Compute the end body point normal acceleration in world coordinate (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyVelB.
		* @param normalAccB Normal bodies acceleration in body frame.
		* @param X_b_p normal acceleration point/frame.
		* @param V_b_p X_b_p velocity.
		* @return End body point normal acceleration in world coordinate.
		*/
	sva::MotionVecd normalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const std::vector<sva::MotionVecd>& normalAccB,
		const sva::PTransformd& X_b_p, const sva::MotionVecd& V_b_p) const;

	/**
		* Compute the end body point normal acceleration in world coordinate (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelW, bodyVelB.
		* @param normalAccB Normal bodies acceleration in body frame.
		* @return End body point normal acceleration in world coordinate.
		*/
	sva::MotionVecd normalAcceleration(const MultiBody& mb,
		const MultiBodyConfig& mbc, const std::vector<sva::MotionVecd>& normalAccB) const;

	/**
		* Compute the end body point normal acceleration in body coordinate (JDotBody·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use nothing.
		* @param normalAccB Normal bodies acceleration in body frame.
		* @return End body point normal acceleration in body coordinate.
		*/
	sva::MotionVecd bodyNormalAcceleration(const MultiBody& mb,
		const MultiBodyConfig& mbc, const std::vector<sva::MotionVecd>& normalAccB) const;

	/**
		* Translate a jacobian at a given position.
		* @param jac Jacobian to translate.
		* @param mbc Use bodyPosW.
		* @param point Point to translate jacobian.
		* @param res Translated jacobian (must be allocated).
		*/
	void translateJacobian(const Eigen::Ref<const Eigen::MatrixXd>& jac,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
		Eigen::MatrixXd& res);

	/**
		* Translate a jacobian at a given position in body frame.
		* @param jac Jacobian to translate.
		* @param mbc Use bodyPosW.
		* @param point Point to translate jacobian.
		* @param res Translated jacobian (must be allocated).
		*/
	void translateBodyJacobian(const Eigen::Ref<const Eigen::MatrixXd>& jac,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
		Eigen::MatrixXd& res);

	/**
		* Project the jacobian in the full robot parameters vector.
		* @param mb MuliBody used has model.
		* @param jac Jacobian to project.
		* @param res Projected Jacobian (must be allocated).
		*/
	void fullJacobian(const MultiBody& mb,
		const Eigen::Ref<const Eigen::MatrixXd>& jac,
		Eigen::MatrixXd& res) const;

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
		return static_cast<int>(jac_.cols());
	}

	/// @return Static translation in the body exprimed in body coordinate.
	const Eigen::Vector3d& point() const
	{
		return point_.translation();
	}

	/// @param point Static translation in the body exprimed in body coordinate.
	void point(const Eigen::Vector3d& point)
	{
		point_ = sva::PTransformd(point);
	}


	// safe version for python binding

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc,
		const sva::PTransformd& X_0_p);

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see bodyJacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sBodyJacobian(const MultiBody& mb, const MultiBodyConfig& mbc);

	/** safe version of @see vectorJacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sVectorJacobian(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& vec);

	/** safe version of @see vectorBodyJacobian.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sVectorBodyJacobian(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& vec);

	/** safe version of @see subMultiBody.
		* @throw std::domain_error If mb don't match jointPath.
		*/
	MultiBody sSubMultiBody(const MultiBody& mb) const;

	/** safe version of @see jacobianDot.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sJacobianDot(const MultiBody& mb,
		const MultiBodyConfig& mbc);

	/** safe version of @see bodyJacobianDot.
		* @throw std::domain_error If mb don't match mbc or jointPath.
		*/
	const Eigen::MatrixXd& sBodyJacobianDot(const MultiBody& mb,
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
		Eigen::MatrixXd& res) const;

	/** safe version of @see velocity.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::MotionVecd sVelocity(const MultiBody& mb, const MultiBodyConfig& mbc,
		const sva::PTransformd& X_b_p) const;

	/** safe version of @see velocity.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::MotionVecd sVelocity(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/** safe version of @see normalAcceleration.
		* @throw std::domain_error If mb don't match mbc or normalAccB.
		*/
	sva::MotionVecd sNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const sva::PTransformd& X_b_p, const sva::MotionVecd& V_b_p) const;

	/** safe version of @see normalAcceleration.
		* @throw std::domain_error If mb don't match mbc or normalAccB.
		*/
	sva::MotionVecd sNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const std::vector<sva::MotionVecd>& normalAccB,
		const sva::PTransformd& X_b_p, const sva::MotionVecd& V_b_p) const;

	/** safe version of @see normalAcceleration.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::MotionVecd sNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/** safe version of @see bodyVelocity.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::MotionVecd sBodyVelocity(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/** safe version of @see bodyNormalAcceleration.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::MotionVecd sBodyNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc) const;

	/** safe version of @see normalAcceleration.
		* @throw std::domain_error If mb don't match mbc or normalAccB.
		*/
	sva::MotionVecd sNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const std::vector<sva::MotionVecd>& normalAccB) const;

	/** safe version of @see bodyNormalAcceleration.
		* @throw std::domain_error If mb don't match mbc or normalAccB.
		*/
	sva::MotionVecd sBodyNormalAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc,
		const std::vector<sva::MotionVecd>& normalAccB) const;

private:
	sva::MotionVecd normalAcceleration(const MultiBodyConfig& mbc,
		const sva::MotionVecd& bodyNNormalAcc, const sva::PTransformd& X_b_p,
		const sva::MotionVecd& V_b_p) const;
	sva::MotionVecd normalAcceleration(const MultiBodyConfig& mbc,
		const sva::MotionVecd& bodyNNormalAcc) const;
	sva::MotionVecd bodyNormalAcceleration(const MultiBodyConfig& mbc,
		const sva::MotionVecd& bodyNNormalAcc) const;

private:
	std::vector<int> jointsPath_;
	sva::PTransformd point_;

	Eigen::MatrixXd jac_;
	Eigen::MatrixXd jacDot_;
};

} // namespace rbd

