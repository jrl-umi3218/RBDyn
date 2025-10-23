/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/FD.h"

// includes
// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

ForwardDynamics::ForwardDynamics(const MultiBody & mb)
: H_(mb.nrDof(), mb.nrDof()), C_(mb.nrDof()), I_st_(static_cast<size_t>(mb.nrBodies())),
  F_(static_cast<size_t>(mb.nrJoints())), HIr_(mb.nrDof(), mb.nrDof()), acc_(static_cast<size_t>(mb.nrBodies())),
  f_(static_cast<size_t>(mb.nrBodies())), tmpFd_(mb.nrDof()), dofPos_(static_cast<size_t>(mb.nrJoints())),
  ldlt_(mb.nrDof())
{
  HIr_.setZero();
  int dofP = 0;
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    F_[ui].resize(6, mb.joint(i).dof());
    dofPos_[ui] = dofP;
    dofP += mb.joint(i).dof();
  }
  computeHIr(mb);
}

void ForwardDynamics::forwardDynamics(const MultiBody & mb, MultiBodyConfig & mbc)
{
  computeH(mb, mbc);
  computeC(mb, mbc);

  paramToVector(mbc.jointTorque, tmpFd_);
  ldlt_.compute(H_);
  tmpFd_ = ldlt_.solve(tmpFd_ - C_);

  vectorToParam(tmpFd_, mbc.alphaD);
}

void ForwardDynamics::computeHIr(const MultiBody & mb)
{
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    if(mb.joint(i).type() == Joint::Rev)
    {
      double gr = mb.joint(i).gearRatio();
      HIr_(dofPos_[ui], dofPos_[ui]) = mb.joint(i).rotorInertia() * gr * gr;
    }
  }
}

void ForwardDynamics::computeH(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  H_.setZero();
  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    I_st_[i] = bodies[i].inertia();
  }

  for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    if(pred[ui] != -1)
    {
      const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
      I_st_[static_cast<size_t>(pred[ui])] += X_p_i.transMul(I_st_[ui]);
    }

    for(int dof = 0; dof < joints[ui].dof(); ++dof)
    {
      F_[ui].col(dof).noalias() = (I_st_[ui] * sva::MotionVecd(mbc.motionSubspace[ui].col(dof))).vector();
    }

    H_.block(dofPos_[ui], dofPos_[ui], joints[ui].dof(), joints[ui].dof()).noalias() =
        mbc.motionSubspace[ui].transpose() * F_[ui];

    size_t j = ui;
    while(pred[j] != -1)
    {
      const sva::PTransformd & X_p_j = mbc.parentToSon[j];
      for(int dof = 0; dof < joints[ui].dof(); ++dof)
      {
        F_[ui].col(dof) = X_p_j.transMul(sva::ForceVecd(F_[ui].col(dof))).vector();
      }
      j = static_cast<size_t>(pred[j]);

      if(joints[j].dof() != 0)
      {
        H_.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).noalias() =
            F_[ui].transpose() * mbc.motionSubspace[j];

        H_.block(dofPos_[j], dofPos_[ui], joints[j].dof(), joints[ui].dof()).noalias() =
            H_.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).transpose();
      }
    }
  }

  H_.noalias() = H_ + HIr_;
}

void ForwardDynamics::computeC(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  const std::vector<Body> & bodies = mb.bodies();
  const std::vector<Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  sva::MotionVecd a_0(Eigen::Vector3d::Zero(), mbc.gravity);

  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    const sva::PTransformd & X_p_i = mbc.parentToSon[i];

    const sva::MotionVecd & vj_i = mbc.jointVelocity[i];

    const sva::MotionVecd & vb_i = mbc.bodyVelB[i];

    if(pred[i] != -1)
      acc_[i] = X_p_i * acc_[static_cast<size_t>(pred[i])] + vb_i.cross(vj_i);
    else
      acc_[i] = X_p_i * a_0 + vb_i.cross(vj_i);

    f_[i] = bodies[i].inertia() * acc_[i] + vb_i.crossDual(bodies[i].inertia() * vb_i)
            - mbc.bodyPosW[i].dualMul(mbc.force[i]);
  }

  for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    C_.segment(dofPos_[ui], joints[ui].dof()).noalias() = mbc.motionSubspace[ui].transpose() * f_[ui].vector();

    if(pred[ui] != -1)
    {
      const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
      f_[static_cast<size_t>(pred[ui])] += X_p_i.transMul(f_[ui]);
    }
  }
}

void ForwardDynamics::sForwardDynamics(const MultiBody & mb, MultiBodyConfig & mbc)
{
  checkMatchParentToSon(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);
  checkMatchJointVelocity(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchBodyPos(mb, mbc);
  checkMatchForce(mb, mbc);
  checkMatchJointTorque(mb, mbc);

  checkMatchAlphaD(mb, mbc);

  forwardDynamics(mb, mbc);
}

void ForwardDynamics::sComputeH(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchParentToSon(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  computeH(mb, mbc);
}

void ForwardDynamics::sComputeC(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  checkMatchParentToSon(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);
  checkMatchJointVelocity(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchBodyPos(mb, mbc);
  checkMatchForce(mb, mbc);

  computeC(mb, mbc);
}

} // namespace rbd
