/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/Friction.h"

#define EPSILON 1E-6

namespace rbd
{

Friction::Friction(const MultiBody & mb)
  : Fr_(mb.nrDof()), dofPos_(mb.nrJoints())
{
  int dofP = 0;
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    dofPos_[i] = dofP;
    dofP += mb.joint(i).dof();
  }
}

StaticModelFriction::StaticModelFriction(const MultiBody & mb)
  : Friction(mb)
{}
  
void StaticModelFriction::computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  Fr_.setZero();
  
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    if(mb.joint(i).type() == Joint::Rev)
    {
      double Ts = mb.joint(i).staticFriction();
      double Tc = mb.joint(i).kineticFriction();
      double Tv = mb.joint(i).viscousFrictionCoeff();

      double wbrk = mb.joint(i).breakawayVelocity();
      
      double w = mbc.alpha[i][0];
      unsigned short sign = w > EPSILON ? 1 : (w < -EPSILON ? -1 : 0);
      
      Fr_(dofPos_[i]) =
	(Ts - Tc) * exp(-abs(w / wbrk)^2) * sign +
	Tc * sign + Tv * w;
    }
  }
}

StribeckModelFriction::StribeckModelFriction(const MultiBody & mb)
  : Friction(mb)
{}
  
void StribeckModelFriction::computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  Fr_.setZero();
  
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    if(mb.joint(i).type() == Joint::Rev)
    {
      double Ts = mb.joint(i).staticFriction();
      double Tc = mb.joint(i).kineticFriction();
      double Tv = mb.joint(i).viscousFrictionCoeff();

      double wbrk = mb.joint(i).breakawayVelocity();
      double wst = wbrk * sqrt(2);
      double wcoul = wbrk / 10;
      
      double w = mbc.alpha[i][0];
      
      Fr_(dofPos_[i]) =
	sqrt(2 * exp(1)) * (Ts - Tc) * exp(-abs(w / wst)^2) * (w / wst) +
	Tc * tanh(w / wcoul) + Tv * w;
    }
  }
}

ImplEulerIntModelFriction::ImplEulerIntModelFriction(const MultiBody & mb, double dt)
  : Friction(mb), dt_(dt), e_(Eigen::VectorXd::Zero(mb.nrDof())),
    Kf_(25), Bf_(5)
{}

void ImplEulerIntModelFriction::computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  Fr_.setZero();
  
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    if(mb.joint(i).type() == Joint::Rev)
    {
      double Ts = mb.joint(i).staticFriction();
      double Tc = mb.joint(i).kineticFriction();
      double Tv = mb.joint(i).viscousFrictionCoeff();

      double w = mbc.alpha[i][0];

      double w_ast = w + (Kf_ * e_(dofPos_[i])) / (Kf_ * dt_ + Bf_);
      double T_ast = (Kf_ * dt_ + Bf_) * w_ast;

      if (abs(w) <= EPSILON)
	Fr_(dofPos_[i]) = T_ast > Ts ? Ts : (T_ast < -Ts ? -Ts : T_ast);
      else
	Fr_(dofPos_[i]) = w > EPSILON ? Tc : (w < -EPSILON ? -Tc : 0.0);  // 0.0 should not occur
      
      e_(dofPos_[i]) = (Bf_ * e_(dofPos_[i]) + dt_ * Fr_(dofPos_[i])) / (Kf_ * dt_ + Bf_);

      Fr_(dofPos_[i]) += Tv * w;
    }
  }
}
  
} // namespace rbd
