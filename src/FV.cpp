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
#include "FV.h"

// includes
#include <iostream>
// RBdyn
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
namespace rbd
{

void forwardVelocity(const MultiBody& mb, MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();
	const std::vector<int>& succ = mb.successors();

	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		const sva::PTransform& X_i = mbc.jointConfig[i];
		const sva::PTransform& X_p_i = mbc.parentToSon[i];

		mbc.jointVelocity[i] = X_i*joints[i].motion(mbc.alpha[i]);

		for(int k = 0; k < joints[i].dof(); ++k)
		{
			mbc.motionSubspace[i].col(k) = (X_i*
				sva::MotionVec(joints[i].motionSubspace().col(k))).vector();
		}

		if(pred[i] != -1)
			mbc.bodyVelB[succ[i]] = X_p_i*mbc.bodyVelB[pred[i]] + mbc.jointVelocity[i];
		else
			mbc.bodyVelB[succ[i]] = mbc.jointVelocity[i];

		sva::PTransform X_0_i(mbc.bodyPosW[succ[i]].rotation());
		mbc.bodyVelW[i] = X_0_i.inv()*mbc.bodyVelB[i];
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
