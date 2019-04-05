/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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
