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
#include "RBDyn/ZMP.h"

// RBDyn
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

Eigen::Vector3d computeCentroidalZMP(MultiBodyConfig& mbc, Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude)
{
	double g;
	Eigen::Vector3d zmp;
	g = mbc.gravity(2);
	zmp(0) = com(0) - ((com(2) - altitude)*comA(0))/(comA(2)+g);
	zmp(1) = com(1) - ((com(2) - altitude)*comA(1))/(comA(2)+g);
	zmp(2) = altitude;
	return zmp;
}

Eigen::Vector3d computeCentroidalZMPNoGravity(Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude)
{
	Eigen::Vector3d zmp;
	zmp(0) = com(0) - ((com(2) - altitude)*comA(0))/(comA(2));
	zmp(1) = com(1) - ((com(2) - altitude)*comA(1))/(comA(2));
	zmp(2) = altitude;
	return zmp;
}

Eigen::Vector3d computeCentroidalZMPComplete(MultiBodyConfig& mbc, Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude, sva::ForceVecd wr_external, double mass)
{
	Eigen::Vector3d zmp;
	double g;
	g = mbc.gravity(2);
	double denom;
	denom = mass*g - wr_external.force()(2);
	zmp(0) = com(0) - ((com(2) - altitude)*comA(0)*(mass*g/denom))/(comA(2))
		+ wr_external.couple()(1)/denom + com(2)*wr_external.force()(0)/denom;
	zmp(1) = com(1) - ((com(2) - altitude)*comA(1)*(mass*g/denom))/(comA(2))
		- wr_external.couple()(0)/denom + com(2)*wr_external.force()(1)/denom;
	zmp(2) = altitude;
	return zmp;
}

}
