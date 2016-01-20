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
#include "VisServo.h"

namespace rbd
{

Eigen::MatrixXd imagePointJacobian(const Eigen::Vector2d& point2d, const double depthEstimate)
{
	Eigen::MatrixXd m(2,6);
	m << point2d[0]*point2d[1], -(1+point2d[0]*point2d[0]),  point2d[1], -1./depthEstimate, 0., point2d[0]/depthEstimate,
			 1+point2d[1]*point2d[1],   -point2d[0]*point2d[1], -point2d[0], 0., -1./depthEstimate, point2d[1]/depthEstimate;
	return m;
}

Eigen::MatrixXd imagePointJacobian(const Eigen::Vector3d& point3d)
{
	// normalized image coordinates
	double x = point3d[0] / point3d[2];
	double y = point3d[1] / point3d[2];
	Eigen::MatrixXd m(2,6);
	m << x*y, -(1+x*x),  y, -1./point3d[2], 0., x/point3d[2],
			 1+y*y,   -x*y, -x, 0., -1./point3d[2], y/point3d[2];
	return m;
}

Eigen::MatrixXd poseJacobian(const Eigen::Matrix3d& rotation, const double rot_angle_threshold)
{
	Eigen::MatrixXd m(6,6);
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
		axis_antisym << 0., -rot_axis(2), rot_axis(1),
										rot_axis(2), 0., -rot_axis(0),
										-rot_axis(1), rot_axis(0), 0.;

		L_theta_u = i3 - rot_angle*0.5*axis_antisym + (1-(sinc_part))*axis_antisym*axis_antisym;
	}
	m << L_theta_u, Eigen::Matrix3d::Zero(),
			 Eigen::Matrix3d::Zero(), rotation.transpose();
	return m;
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
}
