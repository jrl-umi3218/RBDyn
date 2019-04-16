// This file is part of RBDyn.
//
// Copyright (C) 2012-2017 CNRS-AIST JRL, CNRS-UM LIRMM
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
struct MultiBodyConfig;

namespace ik {

static constexpr int MAX_ITERATIONS = 50;
static constexpr double ALPHA = 0.1;
static constexpr double MIN_COST_VALUE = 1e-8;
static constexpr double ALMOST_ZERO = 1e-12; // = Eigen::NumTraits<double>::dummy_precision();
static constexpr double MAX_TIME = 0.1;

/// @brief Specify ik parameters for the solver.
struct IKParams {
		int maxIteration = ik::MAX_ITERATIONS; ///< Maximum number of iterations. Value <= 0 will prevent stops on this criterion.
		double alpha = ik::ALPHA; ///< Learning rate. Must be ALMOST_ZERO < alpha < 1-ALMOST_ZERO.
		double minCostValue = ik::MIN_COST_VALUE; ///< Stopping criterion. Value <= 0 will prevent stops on this criterion.
		double roundThreshold = ik::ALMOST_ZERO; ///< Rounding threshold for the Jacobian. Values <= 0 will do no roundings.
		double maxTime = ik::MAX_TIME; ///< Time stopping criterion. Value <= 0 will prevent stops on this criterion.
};

/// @brief Specify ik solve returned flags.
enum class Flag {
	NoInit, ///< Solve function not yet called.
	MaxIter, ///< Solver stops because maximum of iteration has been reached.
	Optimum, ///< Solver found optimal value.
	Timed ///< Solver timed out.
};

} // ik

/**
	* Inverse Kinematics algorithm.
	*/
class RBDYN_DLLAPI InverseKinematics
{
public:
	enum class ConstraintType {
		Position,
		Orientation,
		Full
	};

public:
	/**
	 * @param mb MultiBody associated with this algorithm.
	 * @param params IK parameters for the solver.
	 */
	InverseKinematics(const MultiBody& mb, ik::IKParams params = {});

	/**
		* Add a constraint to the problem.
		* @param mb MultiBody used has model.
		* @param bodyName Name of the body the constraint is attached to.
		* @param endEffector Fframe attached to the body to manipulate in relative coordinates.
		* @param target Frame the endEffector needs to reach in world coordiantes.
		* @param type Constraint type. Default is Full.
		*/
	void addConstraint(const MultiBody& mb, const std::string& bodyName, const sva::PTransformd& endEffector, 
		const sva::PTransformd& target, ConstraintType type = ConstraintType::Full);
	/**
		* Add a constraint of type Position to the problem.
		* @param mb MultiBody used has model.
		* @param bodyName Name of the body the constraint is attached to.
		* @param endEffector Fframe attached to the body to manipulate in relative coordinates.
		* @param target Frame the endEffector needs to reach in world coordiantes.
		*/
	void addConstraint(const MultiBody& mb, const std::string& bodyName, 
		const sva::PTransformd& endEffector, const Eigen::Vector3d& target);
	/**
		* Add a constraint of type Orientation to the problem.
		* @param mb MultiBody used has model.
		* @param bodyName Name of the body the constraint is attached to.
		* @param endEffector Fframe attached to the body to manipulate in relative coordinates.
		* @param target Frame the endEffector needs to reach in world coordiantes.
		*/
	void addConstraint(const MultiBody& mb, const std::string& bodyName, 
		const sva::PTransformd& endEffector, const Eigen::Matrix3d& target);
	/// @brief Get IK parameters.
	const ik::IKParams& ikParams() const { return ikParams_; }
	/// @brief Set IK parameters (filter off values).
	void ikParams(const ik::IKParams& ikParams) { ikParams_ = filterParams(ikParams); }
	/// @brief Get solving flag.
	ik::Flag solveFlag() const { return flag_; };
	/// @brief Get time to solve.
	double solveTime() const { return solveTime_; }
	/// @brief Get number of iteration to solve.
	int solveIter() const { return solveIter_; }
	/**
		* Compute the inverse kinematics.
		* @param mb MultiBody used has model.
		* @param mbc Use q generalized position vector
		* @return bool if computation has converged
		* Fill q with new generalized position, update bodyPosW,
		* jointConfig and parentToSon. All computations are done
		* in-place : even if computation does not converge,
		* mbc will be modified.
		*/
	bool inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc);

	/** safe version of @see inverseKinematics.
		* @throw std::domain_error If mb doesn't match mbc.
		*/
	bool sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc);
	/** safe version of @see ikParams.
		* @throw std::domain_error If alpha not set between ALMOST_ZERO and 1-ALMOST_ZERO.
		*/
	void sIKParams(const ik::IKParams& ikParams);


private:
	struct IKConstraint {
		int bodyIndex;
		int posInFulljac;
		int nrConstr;
		ConstraintType type;
		sva::PTransformd endEffector;
		sva::PTransformd target;
		Jacobian jac;
	};

private:
	void computeAb(const MultiBody& mb, const MultiBodyConfig& mbc);
	ik::IKParams filterParams(const ik::IKParams& ikParams) const;

private:
	// @brief ef_index is the End Effector index used to build jacobian
	ik::IKParams ikParams_;
	ik::Flag flag_;
	int nrConstraint_;
	std::vector<IKConstraint> constraints_;
	Eigen::MatrixXd A_;
	Eigen::VectorXd b_;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd_;

	int solveIter_;
	double solveTime_;
};

} // namespace rbd
