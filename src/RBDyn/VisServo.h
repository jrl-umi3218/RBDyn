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
// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include <rbdyn/config.hh>

namespace rbd
{
/**
	* Compute the interaction matrix of an image point
	*
	* @param point2d normalized image coordinates (x,y) = (X/Z, Y/Z)
	* @param depthEstimate an estimate of the point depth Z
	* @param jac is the output Jacobian
	*/
RBDYN_DLLAPI void imagePointJacobian(const Eigen::Vector2d& point2d, const double depthEstimate, Eigen::Matrix<double, 2, 6>& jac);

/**
	* Compute the interaction matrix of an image point
	*
	* @param point3d metric location of the point relative to the camera frame
	* @param jac is the output Jacobian
	*/
RBDYN_DLLAPI void imagePointJacobian(const Eigen::Vector3d& point3d, Eigen::Matrix<double, 2, 6> &jac);

/**
	* Compute the interaction matrix derivative of an image point
	*
	* @param imagePoint is the normalized 2D point
	* @param imagePointSpeed is the speed of the normalized 2D point
	* @param depth is the depth estimate
	* @param depthDot is the derivative of the depth estimate
	* @param jac is the output Jacobian
	*/
RBDYN_DLLAPI void imagePointJacobianDot(const Eigen::Vector2d imagePoint, const Eigen::Vector2d imagePointSpeed, const double depth, const double depthDot, Eigen::Matrix<double, 2, 6> &jac);

/**
	* Compute the interaction matrix of a pose
	*
	* @param rotation matrix
	* @param jac is the output Jacobian
	* @param rot_angle_threshold is the minimum angle of an axis angle representation where the angle
	*		is considered as zero
	*/
RBDYN_DLLAPI void poseJacobian(const Eigen::Matrix3d& rotation, Eigen::Matrix<double, 6, 6>& jac, const double rot_angle_threshold=1.0e-8);

/**
	* Compute the interaction matrix of the depth derivative
	*
	* @param imagePointSpeed is the normalized 2D coordinate speed
	* @param depthEstimate is the estimate of the current depth
	*/
RBDYN_DLLAPI void depthDotJacobian(const Eigen::Vector2d imagePointSpeed, const double depthEstimate, Eigen::Matrix<double, 1, 6>& jac);

/**
	* Compute the angle and axis of an angle-axis rotation representation given a rotation matrix
	*
	* @param rotation matrix as input
	* @param rot_angle as output angle
	* @param rot_axis as output axis
	*/
RBDYN_DLLAPI void getAngleAxis(const Eigen::Matrix3d& rotation, double& rot_angle, Eigen::Vector3d& rot_axis);

/**
	* Obtain the skew-symmetric (or anti-symmetric) matrix of a vector
	*
	* @param vector as input
	* @param matrix as output
	*/
RBDYN_DLLAPI void getSkewSym(const Eigen::Vector3d& vector, Eigen::Matrix3d& matrix);
}
