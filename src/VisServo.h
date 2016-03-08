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
	*/
RBDYN_DLLAPI Eigen::MatrixXd imagePointJacobian(const Eigen::Vector2d& point2d, const double depthEstimate);

/**
	* Compute the interaction matrix of an image point
	*
	* @param point3d metric location of the point relative to the camera frame
	*/
RBDYN_DLLAPI Eigen::MatrixXd imagePointJacobian(const Eigen::Vector3d& point3d);

/**
	* Compute the interaction matrix of a pose
	*
	* @param rotation matrix
	* @param rot_angle_threshold is the minimum angle of an axis angle representation where the angle
	*		is considered as zero
	*/
RBDYN_DLLAPI Eigen::MatrixXd poseJacobian(const Eigen::Matrix3d& rotation, const double rot_angle_threshold=1.0e-8);

/**
	* Compute the angle and axis of an angle-axis rotation representation given a rotation matrix
	*
	* @param rotation matrix as input
	* @param rot_angle as output angle
	* @param rot_axis as output axis
	*/
RBDYN_DLLAPI void getAngleAxis(const Eigen::Matrix3d& rotation, double& rot_angle, Eigen::Vector3d& rot_axis);
}

