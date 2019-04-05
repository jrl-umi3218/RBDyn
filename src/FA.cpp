/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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
