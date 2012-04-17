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

// associated header
#include "EulerIntegration.h"

// includes
#include <iostream>
// RBDyn
#include "MultiBody.h"
#include "MultiBodyConfig.h"

namespace rbd
{

Eigen::Matrix<double, 4, 3> angularVelToQuatVel(const std::vector<double>& q)
{
	return 0.5*(Eigen::Matrix<double, 4, 3>() <<
		-q[1], -q[2], -q[3],
		q[0], -q[3], q[2],
		q[3], q[0], -q[1],
		-q[2], q[1], q[0]).finished();
}



void eulerJointIntegration(Joint::Type type, const std::vector<double>& alpha,
	double step, std::vector<double>& q)
{
	switch(type)
	{
		case Joint::RevX:
		case Joint::RevY:
		case Joint::RevZ:
		case Joint::PrismX:
		case Joint::PrismY:
		case Joint::PrismZ:
		{
			q[0] += alpha[0]*step;
			break;
		}

		case Joint::Free:
		{
			q[4] += alpha[3]*step;
			q[5] += alpha[4]*step;
			q[6] += alpha[5]*step;

			// don't break, we go in spherical
		}
		case Joint::Spherical:
		{
			Eigen::Vector4d qi;
			Eigen::Vector3d w;
			qi << q[0], q[1], q[2], q[3];
			w << alpha[0], alpha[1], alpha[2];

			qi.noalias() += angularVelToQuatVel(q)*w*step;
			qi.normalize();

			q[0] = qi(0);
			q[1] = qi(1);
			q[2] = qi(2);
			q[3] = qi(3);
			break;
		}

		case Joint::Fixed:
		default:
		;
	}
}

void eulerIntegration(const MultiBody& mb, MultiBodyConfig& mbc, double step)
{
	const std::vector<Joint>& joints = mb.joints();

	// integrate
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		eulerJointIntegration(joints[i].type(), mbc.alpha[i], step, mbc.q[i]);
		for(int j = 0; j < joints[i].dof(); ++j)
		{
			mbc.alpha[i][j] += mbc.alphaD[i][j]*step;
		}
	}
}

void sEulerIntegration(const MultiBody& mb, MultiBodyConfig& mbc, double step)
{
	checkMatchQ(mb, mbc);
	checkMatchAlpha(mb, mbc);
	checkMatchAlphaD(mb, mbc);

	eulerIntegration(mb, mbc, step);
}

} // namespace rbd

