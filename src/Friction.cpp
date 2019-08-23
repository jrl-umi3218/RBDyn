/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/Friction.h"

namespace rbd
{

Friction::Friction(const MultiBody & mb) : Fr_(mb.nrDof()), dofPos_(mb.nrJoints())
{
  int dofP = 0;
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    dofPos_[i] = dofP;
    dofP += mb.joint(i).dof();
  }
}

const Eigen::VectorXd & Friction::friction(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  Fr_.setZero();
  
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    if(mb.joint(i).type() == Joint::Rev)
    {
      double Fs = mb.joint(i).staticFriction();
      double Fc = mb.joint(i).kineticFriction();
      double omega_s = mb.joint(i).breakawayVelocity();
      double Fv = mb.joint(i).viscousFrictionCoeff();

      double dq = mbc.alpha[i][0];
      unsigned short sign = dq > 0.0 ? 1 : (dq < 0.0 ? -1 : 0);
      
      Fr_(dofPos_[i]) = (Fc + Fs * exp(-abs(dq / omega_s)^2)) * sign + Fv * dq;
    }
  }

  return Fr_;
};
  
} // namespace rbd
