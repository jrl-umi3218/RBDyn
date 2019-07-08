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

std::pair<Quaterniond, Vector3d> freeJointIntegration_(const Quaterniond& qi, const Vector3d& xi, 
                                                       const Quaterniond& qf,
                                                       Vector3d wi, Vector3d vi, 
                                                       const Vector3d& wD, const Vector3d& vD,
                                                       double step, double prec = 1e-10)
{
  double erri = fourthDerivativeSquaredNorm(vi, wi, vD, wD);
  double errf = fourthDerivativeSquaredNorm(vi+step*vD, wi+step*wD, vD, wD);
  double errMax = std::max(erri, errf);
  int n = static_cast<int>(std::ceil(step / 2 * std::sqrt(std::sqrt(errMax * step / (180 * prec)))));

  double nthStep = step / n;
  double halfNthStep = nthStep / 2;

  std::pair<Quaterniond, Vector3d> H = {qi, 6 * xi + nthStep * (qi * vi)};
  for(int i = 0; i < n - 1; ++i)
  {
    Vector3d vh = vi + vD * halfNthStep;
    Vector3d ve = vi + vD * nthStep;
    Quaterniond qh = rbd::SO3Integration(H.first, wi, wD, halfNthStep).first;
    Quaterniond qe = rbd::SO3Integration(H.first, wi, wD, nthStep).first;
    H.second += nthStep * (4 * (qh * vh) + 2 * (qe * ve));
    H.first = qe;
    vi = ve;
    wi += nthStep * wD;
  }
  Vector3d vh = vi + vD * halfNthStep;
  Vector3d vf = vi + vD * nthStep;
  Quaterniond qh = rbd::SO3Integration(H.first, wi, wD, halfNthStep).first;

  //std::cout << qh.coeffs().transpose() << std::endl;
  //std::cout << rbd::SO3Integration(H.first, wi, wD, nthStep).first.coeffs().transpose() << std::endl;

  H.second += nthStep * (4 * (qh * vh) + qf * vf);
  H.first = qf;
  H.second /= 6;

  return H;
}

std::pair<Quaterniond, Vector3d> freeJointIntegration(const Quaterniond& qi, const Vector3d& xi, 
                                                       const Vector3d& wi, const Vector3d& vi, 
                                                       const Vector3d& wD, const Vector3d& vD,
                                                       double step, double prec = 1e-10)
{
  auto qf = rbd::SO3Integration(qi, wi, wD, step, prec, prec);
  if (!qf.second)
  {
    return freeJointIntegration_(qi, xi, qf.first, wi, vi, wD, vD, step, prec);
  }
  else
  {
    double halfStep = step / 2;
    auto H = freeJointIntegration(qi, xi, wi, vi, wD, vD, halfStep, prec);
    return freeJointIntegration(H.first, H.second, wi + halfStep * wD, vi + halfStep * vD, wD, vD, halfStep, prec);
  }
}

Quaterniond sphericalJointIntegration(const Quaterniond& qi, const Vector3d& wi,
                                      const Vector3d& wD, double step, double prec = 1e-10)
{
  auto qf = rbd::SO3Integration(qi, wi, wD, step, prec, prec);
  if (!qf.second)
  {
    return qf.first;
  }
  else
  {
    double halfStep = step / 2;
    auto q = sphericalJointIntegration(qi, wi, wD, halfStep, prec);
    return sphericalJointIntegration(q, wi + halfStep * wD, wD, halfStep, prec);
  }
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
                           std::vector<double> & q,
                           double prec)
{
  switch(type)
  {
    case Joint::Rev:
    case Joint::Prism:
    {
      double step2 = step * step;
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
      double step2 = step * step;
      q[0] += alpha[0] * step + alphaD[0] * step2 / 2;
      q[1] += alpha[1] * step + alphaD[1] * step2 / 2;
      break;
    }

    /// @todo manage reverse joint
    case Joint::Free:
    {
      Quaterniond qi(q[0], q[1], q[2], q[3]);
      Map<Vector3d> xi(&q[4]);
      Vector3d wi(alpha[0], alpha[1], alpha[2]);
      Vector3d wD(alphaD[0], alphaD[1], alphaD[2]);
      Vector3d vi(alpha[3], alpha[4], alpha[5]);
      Vector3d vD(alphaD[3], alphaD[4], alphaD[5]);

      auto H = freeJointIntegration(qi, xi, wi, vi, wD, vD, step, prec);
      double nq = H.first.norm();

      // Normalization should not be necessary but we keep it for robustness
      q[0] = H.first.w() / nq;
      q[1] = H.first.x() / nq;
      q[2] = H.first.y() / nq;
      q[3] = H.first.z() / nq;

      xi = H.second;
      break;
    }
    /// @todo manage reverse joint
    /* fallthrough */
    case Joint::Spherical:
    {
      Quaterniond qi(q[0], q[1], q[2], q[3]);
      Vector3d wi(alpha[0], alpha[1], alpha[2]);
      Vector3d wD(alphaD[0], alphaD[1], alphaD[2]);

      auto qf = sphericalJointIntegration(qi, wi, wD, step, prec);
      qf.normalize(); // This step should not be necessary but we keep it for robustness

      q[0] = qf.w();
      q[1] = qf.x();
      q[2] = qf.y();
      q[3] = qf.z();
      break;
    }

    case Joint::Fixed:
    default:;
  }
}

void eulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec)
{
  const std::vector<Joint> & joints = mb.joints();

  // integrate
  for(std::size_t i = 0; i < joints.size(); ++i)
  {
    eulerJointIntegration(joints[i].type(), mbc.alpha[i], mbc.alphaD[i], step, mbc.q[i], prec);
    for(size_t j = 0; j < static_cast<size_t>(joints[i].dof()); ++j)
    {
      mbc.alpha[i][j] += mbc.alphaD[i][j] * step;
    }
  }
}

void sEulerIntegration(const MultiBody & mb, MultiBodyConfig & mbc, double step, double prec)
{
  checkMatchQ(mb, mbc);
  checkMatchAlpha(mb, mbc);
  checkMatchAlphaD(mb, mbc);

  eulerIntegration(mb, mbc, step, prec);
}

} // namespace rbd
