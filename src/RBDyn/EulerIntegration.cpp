/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// associated header
#include "RBDyn/EulerIntegration.h"

// includes
// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

#include <iostream>

using namespace Eigen;

namespace
{

/** Compute the sum of the first terms of the Magnus expansion of \Omega such that
 * q' = q*exp(\Omega) is the quaternion obtained after applying a constant acceleration
 * rotation wD for a duration step, while starting with a velocity w
 * The function computes the i first terms of the sum, such that ||O_i+1|| < absEps and
 * ||O_i+1|| < relEps * ||O1||. It stops at the 5th term even if the conditions where not
 * met, in which case the return bool is set to true as a warning.
 */
std::pair<Vector3d, bool> magnusExpansion(const Vector3d & w, const Vector3d & wD, double step,
                                          double relEps, double absEps)
{
  double step2 = step * step;

  Vector3d w1 = w + wD * step;
  Vector3d O1 = (w + w1) * step / 2;
  Vector3d O2 = w.cross(w1) * step2 / 12;

  double sqn1 = O1.squaredNorm();           // ||O1||^2
  double sqn2 = O2.squaredNorm();           // ||O2||^2
  double sqnd = wD.squaredNorm();           // ||wD||^2
  double sqndt4 = sqnd * step2 * step2;     // ||wD||^2 t^4
  double sqn3 = sqndt4 / 400;               // ||O3||^2
  double sqn4 = sqn1 * sqn1 * sqn2 / 3600;  // upper bound for // ||O4||^2
  double eps2 = std::min(relEps * relEps * sqn1, absEps * absEps); // squared absolute error
  
  if (sqn3 < eps2 && sqn4 < eps2)
  {
    return {O1+O2, false};
  }

  Vector3d O3 = wD.cross(O2) * step2 / 20;
  Vector3d O4 = (28 * sqn1 - 3 * sqndt4) / 1680 * O2;

  double sqn5 = (sqndt4 * sqn1 + 8 * std::sqrt(sqndt4 * sqn1 * sqn2) + 16 * sqn2) / (840 * 840);

  if (sqn5 < eps2)
  {
    return {O1 + O2 + O3 + O4, false};
  }

  Vector3d O5 = ((120 * sqn1 - 5 * sqndt4) * O3 - 24 * sqn2 * O1) / 5040;
  return {O1 + O2 + O3 + O4 + O5, true};
}

/** Compute the squared norm of the 4th derivative of f = R(t)v(t), where R is a rotation with
 * speed w and constant acceleration dw and v is a linear velocity with constant accleration dv.
 * Noting u.v the dot product of u and v, and uxv the cross product, we have
 * f^(4) = R((||w||^4 - 3||dw||^2) v - 12 w.dw dv + (4 dw.dv - ||w||^2 w.v) w
 *  + (3 dw.v + 8 w.dv) dw + (5 w.dw v + 4||w||^2 dv) x w + (2 w.v w + ||w||^2 v) x dw)
 * Note that norm is independent of R, because R^T R = I;
 */
double fourthDerivativeSquaredNorm(const Vector3d & v,
                                   const Vector3d & w,
                                   const Vector3d & dv,
                                   const Vector3d & dw)
{
  double nw2 = w.squaredNorm();
  double nw4 = nw2 * nw2;
  double ndw2 = dw.squaredNorm();
  double wv = w.dot(v);
  double wdw = w.dot(dw);
  double dwv = dw.dot(v);
  double wdv = w.dot(dv);
  double dwdv = dw.dot(dv);

  Vector3d u = (nw4 - 3 * ndw2) * v - 12 * wdw * dv + (4 * dwdv - nw2 * wv) * w + (3 * dwv + 8 * wdv) * dw
                      - w.cross(5 * wdw * v + 4 * nw2 * dv) - dw.cross(2 * wv * w + nw2 * v);

  return u.squaredNorm();
}

} // namespace

namespace rbd
{

std::pair<Quaterniond, bool> SO3Integration(const Quaterniond & qi,
                                            const Vector3d & wi,
                                            const Vector3d & wD,
                                            double step,
                                            double relEps,
                                            double absEps,
                                            bool breakOnWarning)
{
  // See https://cwzx.wordpress.com/2013/12/16/numerical-integration-for-rotational-dynamics/
  // the division by 2 is because we want to compute exp(O) = (cos(||O||/2), sin(||O||/2)*O/||O||)
  // in quaternion form.
  auto mag = magnusExpansion(wi, wD, step, relEps, absEps);
  if (breakOnWarning && mag.second)
  {
    return {qi, true};
  }
  Vector3d O = mag.first / 2;
  double n = O.norm();
  double s = sva::sinc(n);
  Quaterniond qexp(std::cos(n), s * O.x(), s * O.y(), s * O.z());

  return {qi * qexp, mag.second};
}

void eulerJointIntegration(Joint::Type type,
                           const std::vector<double> & alpha,
                           const std::vector<double> & alphaD,
                           double step,
                           std::vector<double> & q)
{
  double step2 = step * step;

  // Compute the rotation qf attained after the duration step, starting from qi with speed
  // w0 and constant acceleration wD. The result is also stored in q.
  auto commonSphereFree = [&](Quaterniond & qi, Quaterniond & qf, Vector3d & w0,
                              Vector3d & wD) {
    qi = Quaterniond(q[0], q[1], q[2], q[3]);
    w0 = Vector3d(alpha[0], alpha[1], alpha[2]);
    wD = Vector3d(alphaD[0], alphaD[1], alphaD[2]);

    qf = SO3Integration(qi, w0, wD, step).first;
    qf.normalize(); // This step should not be necessary but we keep it for robustness

    q[0] = qf.w();
    q[1] = qf.x();
    q[2] = qf.y();
    q[3] = qf.z();
  };

  switch(type)
  {
    case Joint::Rev:
    case Joint::Prism:
    {
      q[0] += alpha[0] * step + alphaD[0] * step2 / 2;
      break;
    }

    /// @todo manage reverse joint
    case Joint::Planar:
    {
      // This is the old implementation akin to x' = x + v*step
      // (i.e. we don't take the acceleration into account)
      /// @todo us the acceleration
      double q1Step = q[2] * alpha[0] + alpha[1];
      double q2Step = -q[1] * alpha[0] + alpha[2];
      q[0] += alpha[0] * step;
      q[1] += q1Step * step;
      q[2] += q2Step * step;
      break;
    }

    case Joint::Cylindrical:
    {
      q[0] += alpha[0] * step + alphaD[0] * step2 / 2;
      q[1] += alpha[1] * step + alphaD[1] * step2 / 2;
      break;
    }

    /// @todo manage reverse joint
    case Joint::Free:
    {
      // Rotation part
      Quaterniond qi, qf;
      Vector3d wi, wD;
      commonSphereFree(qi, qf, wi, wD);

      // For the translation part x, we have that \dot{x} = R*v, where v is the translation velocity
      // and R is the orientation part. This is because, due to Featherstone's choices, the velocity
      // and acceleration are in FS coordinate while the position in FP coordinate.
      // We integrate x with Simpson's rule (i.e. RK4 for a case where the function to integrate does
      // not depend on x)
      Vector3d vi(alpha[3], alpha[4], alpha[5]);
      Vector3d a(alphaD[3], alphaD[4], alphaD[5]);
      Vector3d vh = vi + a * step / 2;
      Vector3d vf = vi + a * step;

      Quaterniond qh = rbd::SO3Integration(qi, wi, wD, step / 2).first;

      Vector3d k1 = step * (qi * vi);
      Vector3d k2 = step * (qh * vh);
      Vector3d k4 = step * (qf * vf);

      Map<Vector3d> x(&q[4]);
      x += (k1 + 4 * k2 + k4) / 6;

      //std::cout << (step*step*step*step*step)/2880*fourthDerivativeSquaredNorm(vi, wi, a, wD) << std::endl;

      break;
    }
    /// @todo manage reverse joint
    /* fallthrough */
    case Joint::Spherical:
    {
      Quaterniond qi, qf;
      Vector3d w0, wD;
      commonSphereFree(qi, qf, w0, wD);
      break;
    }

    case Joint::Fixed:
    default:;
  }
}

void eulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step)
{
  const std::vector<Joint> & joints = mb.joints();

  // integrate
  for(std::size_t i = 0; i < joints.size(); ++i)
  {
    eulerJointIntegration(joints[i].type(), mbc.alpha[i], mbc.alphaD[i], step, mbc.q[i]);
    for(size_t j = 0; j < static_cast<size_t>(joints[i].dof()); ++j)
    {
      mbc.alpha[i][j] += mbc.alphaD[i][j] * step;
    }
  }
}

void sEulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step)
{
  checkMatchQ(mb, mbc);
  checkMatchAlpha(mb, mbc);
  checkMatchAlphaD(mb, mbc);

  eulerIntegration(mb, mbc, step);
}

} // namespace rbd
