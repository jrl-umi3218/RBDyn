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

// include
#include <vector>
// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

#include <vector>

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;
struct Block;
using Blocks = std::vector<Block>;
class Jacobian;


/**
	* Compute the centroidal momentum at the CoM frame
	* as describe in [Orin and Gosawami 2008].
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW, bodyVelB.
	* @param com CoM position.
	* @return centroidal momentum at the CoM frame.
	*/
RBDYN_DLLAPI sva::ForceVecd computeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com);


/**
	* @brief Compute the time derivative of centroidal momentum at the CoM frame.
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW, bodyVelB, bodyAccB.
	* @param com CoM position.
	* @param comDot CoM velocity.
	* @return Derivative of the centroidal momentum at the CoM frame.
	*/
RBDYN_DLLAPI sva::ForceVecd computeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot);


/**
	* Compute the Centroidal momentum matrix (Jacobian)
	* as describe in [Orin and Gosawami 2008].
	*/
class RBDYN_DLLAPI CentroidalMomentumMatrix
{
public:
	CentroidalMomentumMatrix();

	/// @param mb MultiBody used has model.
	CentroidalMomentumMatrix(const MultiBody& mb);
	/**
	 * @param mb MultiBody used has model.
	 * @param weight Per body weight.
	 */
	CentroidalMomentumMatrix(const MultiBody &mb, std::vector<double> weight);

public:
	/**
		* Compute the centroidal momentum matrix.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, motionSubspace.
		* @param com CoM position.
		*/
	void computeMatrix(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com);

	/**
		* Compute the time derivative of centroidal momentum matrix.
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelW, bodyVelB, motionSubspace.
		* @param com CoM position.
		* @param comDot CoM velocity.
		*/
	void computeMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com, const Eigen::Vector3d& comDot);

	/**
	 * Compute the centroidal momentum matrix and his time derivative.
	 * @see computeMatrix
	 * @see computeMatrixDot
	 */
	void computeMatrixAndMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com, const Eigen::Vector3d& comDot);

	/// @return Centroidal momentum matrix
	const Eigen::MatrixXd& matrix() const;

	/// @return Centroidal momentum matrix time derivative
	const Eigen::MatrixXd& matrixDot() const;

	/**
		* Compute the centroidal momentum (with weight) (J·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB.
		* @param com CoM position.
		* @return Centroidal momentum at the CoM frame (with weight).
		*/
	sva::ForceVecd momentum(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com) const;

	/**
		* Compute the normal componant of the time derivative of
		* centroidal momentum (with weight) (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB, jointVelocity, parentToSon.
		* @param com CoM position.
		* @param comDot CoM velocity.
		* @return Normal componant of the time derivative Centroidal momentum
		*					at the CoM frame (with weight).
		*/
	sva::ForceVecd normalMomentumDot(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
		const Eigen::Vector3d& comDot);

	/**
		* Compute the normal componant of the time derivative of
		* centroidal momentum (with weight) (JDot·alpha).
		* @param mb MultiBody used has model.
		* @param mbc Use bodyPosW, bodyVelB.
		* @param com CoM position.
		* @param comDot CoM velocity.
		* @param normalAccB Normal bodies acceleration in body frame.
		* @return Normal componant of the time derivative Centroidal momentum
		*					at the CoM frame (with weight).
		*/
	sva::ForceVecd normalMomentumDot(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
		const Eigen::Vector3d& comDot,
		const std::vector<sva::MotionVecd>& normalAccB) const;

	// safe version for python binding

	/** safe version of @see computeMatrix.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeMatrix(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com);

	/** safe version of @see computeMatrixDot.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com, const Eigen::Vector3d& comDot);

	/** safe version of @see computeMatrixAndMatrixDot.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sComputeMatrixAndMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com, const Eigen::Vector3d& comDot);

	/** safe version of @see momentum.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::ForceVecd sMomentum(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com) const;

	/** safe version of @see normalMomentumDot.
		* @throw std::domain_error If mb don't match mbc.
		*/
	sva::ForceVecd sNormalMomentumDot(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
		const Eigen::Vector3d& comDot);

	/** safe version of @see normalMomentumDot.
		* @throw std::domain_error If mb don't match mbc or normalAccB.
		*/
	sva::ForceVecd sNormalMomentumDot(const MultiBody& mb,
		const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
		const Eigen::Vector3d& comDot,
		const std::vector<sva::MotionVecd>& normalAccB) const;

private:
	void init(const rbd::MultiBody& mb);

private:
	Eigen::MatrixXd cmMat_;
	Eigen::MatrixXd cmMatDot_;

	std::vector<Jacobian> jacVec_;
	std::vector<Blocks> blocksVec_;
	std::vector<Eigen::MatrixXd> jacWork_;
	std::vector<double> bodiesWeight_;
	std::vector<sva::MotionVecd> normalAcc_;
};


// safe version for python binding

/**
	* Safe version.
	* @see computeCentroidalMomentum.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
RBDYN_DLLAPI sva::ForceVecd sComputeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com);

/**
	* Safe version.
	* @see computeCentroidalMomentumDot.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
RBDYN_DLLAPI sva::ForceVecd sComputeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot);


} // rbd
