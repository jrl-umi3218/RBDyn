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
#include "RBDyn/FV.h"

// includes
// RBdyn
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

void forwardVelocity(const MultiBody& mb, MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();
	const std::vector<int>& succ = mb.successors();

	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		const sva::PTransformd& X_p_i = mbc.parentToSon[i];

		mbc.jointVelocity[i] = joints[i].motion(mbc.alpha[i]);

		if(pred[i] != -1)
			mbc.bodyVelB[succ[i]] = X_p_i*mbc.bodyVelB[pred[i]] + mbc.jointVelocity[i];
		else
			mbc.bodyVelB[succ[i]] = mbc.jointVelocity[i];

		sva::PTransformd E_0_i(mbc.bodyPosW[succ[i]].rotation());
		mbc.bodyVelW[succ[i]] = E_0_i.invMul(mbc.bodyVelB[succ[i]]);
	}
}

void sForwardVelocity(const MultiBody& mb, MultiBodyConfig& mbc)
{
	checkMatchAlpha(mb, mbc);
	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	checkMatchBodyVel(mb, mbc);
	checkMatchJointVelocity(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	forwardVelocity(mb, mbc);
}

} // namespace rbd
