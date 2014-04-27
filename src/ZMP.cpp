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
#include "ZMP.h"

// RBDyn
#include "MultiBodyConfig.h"

namespace rbd
{

Eigen::Vector3d computeCentroidalZMP(MultiBodyConfig& mbc, Eigen::Vector3d& com,
		Eigen::Vector3d& comA, double altitude)
{
	double g;
	Eigen::Vector3d zmp;
	g = mbc.gravity(2);
	zmp(0) = com(0) - ((com(2) - altitude)*comA(0))/(comA(2)-g);
	zmp(1) = com(1) - ((com(2) - altitude)*comA(1))/(comA(2)-g);
	zmp(2) = altitude;
	return zmp;
}

}
