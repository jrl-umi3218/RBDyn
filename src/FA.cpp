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

// associated header
#include "RBDyn/FA.h"

// includes
// RBdyn
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

void forwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc,
	const sva::MotionVecd& A_0)
{
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();
	const std::vector<int>& succ = mb.successors();

	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		const sva::PTransformd& X_p_i = mbc.parentToSon[i];

		const sva::MotionVecd& vj_i = mbc.jointVelocity[i];
		sva::MotionVecd ai_tan = joints[i].tanAccel(mbc.alphaD[i]);

		const sva::MotionVecd& vb_i = mbc.bodyVelB[i];

		if(pred[i] != -1)
			mbc.bodyAccB[succ[i]] = X_p_i*mbc.bodyAccB[pred[i]] + ai_tan + vb_i.cross(vj_i);
		else
			mbc.bodyAccB[succ[i]] = X_p_i*A_0 + ai_tan + vb_i.cross(vj_i);
	}
}

void sForwardAcceleration(const MultiBody& mb, MultiBodyConfig& mbc,
	const sva::MotionVecd& A_0)
{
	checkMatchAlphaD(mb, mbc);
	checkMatchParentToSon(mb, mbc);
	checkMatchJointVelocity(mb, mbc);
	checkMatchBodyVel(mb, mbc);

	checkMatchBodyAcc(mb, mbc);

	forwardAcceleration(mb, mbc, A_0);
}

} // namespace rbd
