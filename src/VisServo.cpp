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

Eigen::MatrixXd interactionMatrix(const Eigen::Vector2d& point2d, double depthEstimate)
{
	Eigen::MatrixXd m(2,6);
	m << point2d[0]*point2d[1], -(1+point2d[0]*point2d[0]),  point2d[1], -1./depthEstimate, 0., point2d[0]/depthEstimate,
			 1+point2d[1]*point2d[1],   -point2d[0]*point2d[1], -point2d[0], 0., -1./depthEstimate, point2d[1]/depthEstimate;
	return m;
}

Eigen::MatrixXd interactionMatrix(const Eigen::Vector3d& point3d)
{
	// normalized image coordinates
	double x = point3d[0] / point3d[2];
	double y = point3d[1] / point3d[2];
	Eigen::MatrixXd m(2,6);
	m << x*y, -(1+x*x),  y, -1./point3d[2], 0., x/point3d[2],
			 1+y*y,   -x*y, -x, 0., -1./point3d[2], y/point3d[2];
	return m;
}

}
