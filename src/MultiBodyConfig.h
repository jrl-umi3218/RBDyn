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

// sva
#include <SpaceVecAlg>

namespace rbd
{
class MultiBody;

struct MultiBodyConfig
{
	MultiBodyConfig()
	{}
	MultiBodyConfig(const MultiBody& mb);

	/// Generalized position variable.
	std::vector<std::vector<double>> q;

	/// Generalized speed variable.
	std::vector<std::vector<double>> alpha;

	/// Generalized acceleration variable.
	std::vector<std::vector<double>> alphaD;



	/// External force acting on each body in world coordinate.
	std::vector<sva::ForceVec> force;



	/// Joints configuration (Xj).
	std::vector<sva::PTransform> jointConfig;

	/// Joints velocity (Xj*j.motion()).
	std::vector<sva::MotionVec> jointVelocity;

	/// Joints torque.
	std::vector<std::vector<double>> jointTorque;



	/// Motion subspace (Xj.j.subspace).
	std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> motionSubspace;



	/// Bodies transformation in world coordinate.
	std::vector<sva::PTransform> bodyPosW;

	/// Transformation from parent(i) to i in body coordinate (Xj*Xt).
	std::vector<sva::PTransform> parentToSon;



	/// Bodies speed in world coordinate.
	std::vector<sva::MotionVec> bodyVelW;

	/// Bodies speed in Body coordinate.
	std::vector<sva::MotionVec> bodyVelB;

	/// Bodies acceleration in Body coordinate.
	std::vector<sva::MotionVec> bodyAccB;

	/// gravity acting on the multibody.
	Eigen::Vector3d gravity;
};



/**
	* Convert parameter vector to Eigen Vector.
	* @param v Parameter vector.
	* @param e Output Eigen vector (must be of the good size).
	*/
void paramToVector(const std::vector<std::vector<double>>& v, Eigen::VectorXd& e);

/**
	* Safe version of @see paramToVector.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
void sParamToVector(const std::vector<std::vector<double>>& v, Eigen::VectorXd& e);

/**
	* Convert parameter vector to Eigen Vector.
	* @param e Eigen vector.
	* @param e Output Parameter vector (must be of the good size).
	*/
void vectorToParam(const Eigen::VectorXd& e, std::vector<std::vector<double>>& v);

/**
	* Safe version of @see vectorToParam.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
void sVectorToParam(const Eigen::VectorXd& e, std::vector<std::vector<double>>& v);

/// @throw std::domain_error If there is a mismatch between mb and mbc.bodyPosW
void checkMatchBodyPos(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.bodyVelW
void checkMatchBodyVel(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.jointConfig
void checkMatchJointConf(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.q
void checkMatchQ(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.alpha
void checkMatchAlpha(const MultiBody& mb, const MultiBodyConfig& mbc);



} // namespace rbd
