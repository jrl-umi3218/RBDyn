/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <RBDyn/MultiBodyConfig.h>

namespace rbd
{

/**
 * Computation of the feedforward friction vector according
 * to the Stribeck model used in Simulink Simscape
 * https://www.mathworks.com/help/physmod/simscape/ref/rotationalfriction.html
 */
class RBDYN_DLLAPI Friction
{
public:
  /** Initialize the required structures.
   * @param mb Multibody system.
   */
  Friction(const MultiBody & mb);

  /**
   * Compute the feedforward friction vector.
   * @param mb  MultiBody has model.
   * @param mbc MultiBodyConfig has alpha.
   */
  virtual void computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc) = 0;

  /// @return the feedforward friction vector.
  const Eigen::VectorXd & friction() const
  {
    return Fr_;
  }
  
protected:
  Eigen::VectorXd Fr_;

  // friction computation
  std::vector<int> dofPos_;
};

class RBDYN_DLLAPI StaticModelFriction : public Friction
{
public:
  StaticModelFriction(const MultiBody & mb);

  void computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc) override;
};

class RBDYN_DLLAPI StribeckModelFriction : public Friction
{
public:
  StribeckModelFriction(const MultiBody & mb);

  void computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc) override;
};

class RBDYN_DLLAPI ImplEulerIntModelFriction : public Friction
{
public:
  ImplEulerIntModelFriction(const MultiBody & mb, double dt = 0.005);

  void computeFriction(const MultiBody & mb, const MultiBodyConfig & mbc) override;

  void setTimeStep(double dt)
  {
    dt_ = dt;
  }

  void setStaticFrictionStiffness(double Kf)
  {
    Kf_ = Kf;
  }

  double staticFrictionStiffness() const
  {
    return Kf_;
  }
  
  void setStaticFrictionDamping(double Bf)
  {
    Bf_ = Bf;
  }

  double staticFrictionDamping() const
  {
    return Bf_;
  }

private:
  double dt_;
  double Kf_, Bf_;
  Eigen::VectorXd e_;
};
  
}
