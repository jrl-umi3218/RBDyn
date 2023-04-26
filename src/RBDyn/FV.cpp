/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/FV.h"

// includes
// RBdyn
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

void forwardVelocity(const MultiBody & mb, MultiBodyConfig & mbc)
{
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();
  const std::vector<int> & succ = mb.successors();

  for(std::size_t i = 0; i < joints.size(); ++i)
  {
    const auto pred_index = static_cast<size_t>(pred[i]);
    const auto succ_index = static_cast<size_t>(succ[i]);

    const sva::PTransformd & X_p_i = mbc.parentToSon[i];

    mbc.jointVelocity[i] = joints[i].motion(mbc.alpha[i]);

    if(pred[i] != -1)
      mbc.bodyVelB[succ_index] = X_p_i * mbc.bodyVelB[pred_index] + mbc.jointVelocity[i];
    else
      mbc.bodyVelB[succ_index] = mbc.jointVelocity[i];

    sva::PTransformd E_0_i(mbc.bodyPosW[succ_index].rotation());
    mbc.bodyVelW[succ_index] = E_0_i.invMul(mbc.bodyVelB[succ_index]);
  }
}

void sForwardVelocity(const MultiBody & mb, MultiBodyConfig & mbc)
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
