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
#include "RBDyn/FK.h"

// includes
// RBdyn
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

void forwardKinematics(const MultiBody& mb, MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();
	const std::vector<int>& succ = mb.successors();
	const std::vector<sva::PTransformd>& Xt = mb.transforms();

	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		mbc.jointConfig[i] = joints[i].pose(mbc.q[i]);
		mbc.parentToSon[i] = mbc.jointConfig[i]*Xt[i];
		mbc.motionSubspace[i] = joints[i].motionSubspace();

		if(pred[i] != -1)
			mbc.bodyPosW[succ[i]] = mbc.parentToSon[i]*mbc.bodyPosW[pred[i]];
		else
			mbc.bodyPosW[succ[i]] = mbc.parentToSon[i];
	}
}

void sForwardKinematics(const MultiBody& mb, MultiBodyConfig& mbc)
{
	checkMatchQ(mb, mbc);

	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	forwardKinematics(mb, mbc);
}

} // namespace rbd
