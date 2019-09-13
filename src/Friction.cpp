/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/Friction.h"
#include "RBDyn/LambertW/LambertW.h"

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
	(Ts - Tc) * exp(pow(-abs(w / wbrk), 2)) * sign +
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
	sqrt(2 * exp(1)) * (Ts - Tc) * exp(pow(-abs(w / wst), 2)) * (w / wst) +
	Tc * tanh(w / wcoul) + Tv * w;
    }
  }
}

ImplEulerIntModelFriction::ImplEulerIntModelFriction(const MultiBody & mb,
                                                     double Kf, double Bf, double dt)
  : Friction(mb), Kf_(Kf), Bf_(Bf), dt_(dt), e_(Eigen::VectorXd::Zero(mb.nrDof()))
{}

ImplEulerIntModelCoulombFriction::ImplEulerIntModelCoulombFriction(const MultiBody & mb,
                                                                   double Kf, double Bf, double dt)
  : ImplEulerIntModelFriction(mb, Kf, Bf, dt)
{}

void ImplEulerIntModelCoulombFriction::computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc)
{
  Fr_.setZero();
  
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    if(mb.joint(i).type() == Joint::Rev)
    {
      double Tc = mb.joint(i).kineticFriction();
      double Tv = mb.joint(i).viscousFrictionCoeff();

      double w = mbc.alpha[i][0];

      double Z = 1 / (Kf_ * dt_ + Bf_);

      double w_ast = w + Z * Kf_ * e_(dofPos_[i]);
      double T_ast = w_ast / Z;

      double den = 1 + Z * Tv;

      if (T_ast > Tc)
        Fr_(dofPos_[i]) = (Tc + Tv * w) / den;
      else if (T_ast < -Tc)
        Fr_(dofPos_[i]) = (-Tc + Tv * w) / den;
      else
        Fr_(dofPos_[i]) = T_ast;

      e_(dofPos_[i]) = Z * (Bf_ * e_(dofPos_[i]) + Fr_(dofPos_[i]) * dt_);
    }
  }
}

ImplEulerIntModelStictionFriction::ImplEulerIntModelStictionFriction(const MultiBody & mb,
                                                                     double Kf, double Bf, double dt)
  : ImplEulerIntModelFriction(mb, Kf, Bf, dt)
{}

void ImplEulerIntModelStictionFriction::computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc)
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
      
      double Tsc = Ts - Tc;
      
      double w = mbc.alpha[i][0];
      
      double Z = 1 / (Kf_ * dt_ + Bf_);
      
      double w_ast = w + Z * Kf_ * e_(dofPos_[i]);
      double T_ast = w_ast / Z;

      double den = 1 + Z * Tv;
      
      if (T_ast > Ts) {
        double lambArg = -Z/wbrk * Tsc/den * exp(Z/wbrk * (Tc + Tv*w_ast) / den - w_ast/wbrk);
        Fr_(dofPos_[i]) = -(wbrk/Z) * utl::LambertW<0>(lambArg) + (Tc + Tv*w_ast) / den;
      }
      else if (T_ast < -Ts) {
        double lambArg = -Z/wbrk * Tsc/den * exp(-Z/wbrk * (-Tc + Tv*w_ast) / den + w_ast/wbrk);
        Fr_(dofPos_[i]) = (wbrk/Z) * utl::LambertW<0>(lambArg) + (-Tc + Tv*w_ast) / den;
      }
      else
        Fr_(dofPos_[i]) = T_ast;

      e_(dofPos_[i]) = Z * (Bf_ * e_(dofPos_[i]) + Fr_(dofPos_[i]) * dt_);
    }
  }
}
  
} // namespace rbd
