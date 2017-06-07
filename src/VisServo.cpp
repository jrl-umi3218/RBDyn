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

// include
// Associated header
#include "RBDyn/VisServo.h"

namespace rbd
{

void imagePointJacobian(const Eigen::Vector2d& point2d, const double depthEstimate, Eigen::Matrix<double, 2, 6>& jac)
{
	jac << point2d[0]*point2d[1], -(1+point2d[0]*point2d[0]),  point2d[1], -1./depthEstimate, 0., point2d[0]/depthEstimate,
				 1+point2d[1]*point2d[1],   -point2d[0]*point2d[1], -point2d[0], 0., -1./depthEstimate, point2d[1]/depthEstimate;
}

void imagePointJacobian(const Eigen::Vector3d& point3d, Eigen::Matrix<double, 2, 6>& jac)
{
	// convert 3D to 2D normalized image coordinates
	Eigen::Vector2d point2d;
	point2d << point3d[0] / point3d[2], point3d[1] / point3d[2];

	imagePointJacobian(point2d, point3d[2], jac);
}

void imagePointJacobianDot(const Eigen::Vector2d imagePoint, const Eigen::Vector2d imagePointSpeed, const double depth, const double depthDot, Eigen::Matrix<double, 2, 6>& jac)
{
	double Z_sq = (depth*depth);
	jac << imagePointSpeed(0)*imagePoint(1) + imagePoint(0)*imagePointSpeed(1), -2*imagePoint(0)*imagePointSpeed(0), imagePointSpeed(1), depthDot/Z_sq, 0., (imagePointSpeed(0)*depth - imagePoint(0)*depthDot)/Z_sq,
				 2*imagePoint(1)*imagePointSpeed(1), -imagePointSpeed(0)*imagePoint(1)-imagePoint(0)*imagePointSpeed(1), -imagePointSpeed(0), 0., depthDot/Z_sq, (imagePointSpeed(1)*depth - imagePoint(1)*depthDot)/Z_sq;
}

void poseJacobian(const Eigen::Matrix3d& rotation, Eigen::Matrix<double, 6, 6>& jac, const double rot_angle_threshold)
{
	Eigen::Matrix3d i3 = Eigen::Matrix3d::Identity();

	// convert rotation matrix into axis-angle representation
	double rot_angle;
	Eigen::Vector3d rot_axis;
	getAngleAxis(rotation.transpose(), rot_angle, rot_axis);

	// create the rotation jacobian
	Eigen::Matrix3d L_theta_u;

	if(fabs(rot_angle) < rot_angle_threshold)
	{
		// Jacobian simplifies to Identity if the angle is zero, avoids division by zero in sinc
		L_theta_u = i3;
	}
	else
	{
		double sinc_part;
		sinc_part = ((std::sin(rot_angle))/rot_angle)/(std::pow(((std::sin(rot_angle/2.))/(rot_angle/2.)),2));

		Eigen::Matrix3d axis_antisym;
		getSkewSym(rot_axis, axis_antisym);

		L_theta_u = i3 - rot_angle*0.5*axis_antisym + (1-(sinc_part))*axis_antisym*axis_antisym;
	}
	jac << L_theta_u, Eigen::Matrix3d::Zero(),
				 Eigen::Matrix3d::Zero(), rotation.transpose();
}

void depthDotJacobian(const Eigen::Vector2d imagePointSpeed, const double depthEstimate, Eigen::Matrix<double, 1, 6>& jac)
{
	jac << 0., 0., -1., -imagePointSpeed(1)*depthEstimate, imagePointSpeed(0)*depthEstimate, 0.;
}

void getAngleAxis(const Eigen::Matrix3d& rotation, double& rot_angle, Eigen::Vector3d& rot_axis)
{
	// TODO: try to use a better conversion, this is based on the sva::rotationVelocity implementation which has known issues
	double acosV = (rotation(0,0) + rotation(1,1) + rotation(2,2) - 1.)*0.5;
	rot_angle = std::acos(std::min(std::max(acosV,-1.),1.));
	rot_axis << -rotation(2,1) + rotation(1,2),
							-rotation(0,2) + rotation(2,0),
							-rotation(1,0) + rotation(0,1);
}


void getSkewSym(const Eigen::Vector3d& vector, Eigen::Matrix3d& matrix)
{
	matrix <<  0.,				-vector(2),  vector(1),
						 vector(2), 0.,					-vector(0),
						-vector(1), vector(0),	 0.;
}
}
