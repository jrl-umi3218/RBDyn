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
// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{
class MultiBody;
class MultiBodyConfig;
class Jacobian;


/**
	* Compute the centroidal momentum at the CoM frame
	* as describe in [Orin and Gosawami 2008].
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW, bodyVelB.
	* @param com CoM position.
	* @return centroidal momentum at the CoM frame.
	*/
sva::ForceVecd computeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com);


/**
	* @brief Compute the time derivative of centroidal momentum at the CoM frame.
	* @param mb MultiBody used has model.
	* @param mbc Use bodyPosW, bodyVelB, bodyAccB.
	* @param com CoM position.
	* @param comDot CoM velocity.
	* @return Derivative of the centroidal momentum at the CoM frame.
	*/
sva::ForceVecd computeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot);


/**
	* Compute the Centroidal momentum matrix (Jacobian)
	* as describe in [Orin and Gosawami 2008].
	*/
class CentroidalMomentumMatrix
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
	const Eigen::MatrixXd& matrix(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com);

	const Eigen::MatrixXd& matrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
		const Eigen::Vector3d& com);

	// safe version for python binding

	/** safe version of @see jacobian.
		* @throw std::domain_error If mb don't match mbc.
		*/
	const Eigen::MatrixXd& sMatrix(const MultiBody& mb, const MultiBodyConfig& mbc);

private:
	void init(const rbd::MultiBody& mb);

private:
	Eigen::MatrixXd cmMat_;
	Eigen::MatrixXd cmMatDot_;

	Eigen::MatrixXd jacFull_;

	std::vector<Jacobian> jacVec_;
	std::vector<Eigen::MatrixXd> jacWork_;
	std::vector<double> bodiesWeight_;
};


// safe version for python binding

/**
	* Safe version.
	* @see computeCentroidalMomentum.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
sva::ForceVecd sComputeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com);

/**
	* Safe version.
	* @see computeCentroidalMomentumDot.
	* @throw std::domain_error If there is a mismatch between mb and mbc.
	*/
sva::ForceVecd sComputeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comVel);


} // rbd
