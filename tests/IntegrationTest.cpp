/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>
#include <vector>

// boost
#define BOOST_TEST_MODULE JointTest
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/EulerIntegration.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"

using namespace Eigen;
using namespace sva;
using namespace rbd;
namespace constants = boost::math::constants;

/** Timesteps used in the tests*/
const std::vector<double> dt = {0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1};
/** Joints tested*/
const std::vector<Joint::Type> types = {Joint::Rev, Joint::Prism, Joint::Spherical, Joint::Cylindrical, Joint::Free};

/// @return A robot with a single joint specified by the user
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph> makeSingleJointRobot(
    Joint::Type type,
    const Vector3d & axis = Vector3d::UnitZ(),
    const Vector3d & endEffector = Vector3d::Zero())
{
  MultiBodyGraph mbg;

  double mass = 1.;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();
  RBInertiad rbi(mass, h, I);

  Body b0(rbi, "b0");
  Body b1(rbi, "b1");
  mbg.addBody(b0);
  mbg.addBody(b1);

  Joint j0(type, axis, true, "j0");
  mbg.addJoint(j0);

  PTransformd to(Vector3d(0., 0., 0.));
  PTransformd from(Vector3d(0., 0., 0.));
  mbg.linkBodies("b0", to, "b1", from, "j0");

  if (!endEffector.isZero())
  {
    Body ee(rbi, "ee");
    mbg.addBody(ee);
    Joint je(Joint::Fixed, true, "je");
    mbg.addJoint(je);
    mbg.linkBodies("b1", endEffector, "ee", from, "je");
  }

  MultiBody mb = mbg.makeMultiBody("b0", true);
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  return std::make_tuple(mb, mbc, mbg);
}

std::vector<double> randVec(int size, double rmin, double rmax, bool normed = false)
{
  std::vector<double> v(static_cast<size_t>(size), 0);
  Map<VectorXd> r(&v[0], size, 1);
  r = (rmax - rmin) * VectorXd::Random(size).cwiseAbs() + VectorXd::Constant(size, rmin);

  if(normed) r.normalize();

  return v;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> randQVA(Joint::Type type)
{
  std::vector<double> q, v, a;
  switch(type)
  {
    case Joint::Rev:
    case Joint::Prism:
      q = randVec(1, -1, 1);
      v = randVec(1, -1, 1);
      a = randVec(1, -1, 1);
      break;
    case Joint::Spherical:
      q = randVec(4, -1, 1, true);
      v = randVec(3, -1, 1);
      a = randVec(3, -1, 1);
      break;
    case Joint::Planar:
      q = randVec(3, -1, 1);
      v = randVec(3, -1, 1);
      a = randVec(3, -1, 1);
      break;
    case Joint::Cylindrical:
      q = randVec(2, -1, 1);
      v = randVec(2, -1, 1);
      a = randVec(2, -1, 1);
      break;
    case Joint::Free:
    {
      q = randVec(4, -1, 1, true);
      auto qt = randVec(3, -1, 1);
      q.insert(q.end(), qt.begin(), qt.end());
      v = randVec(6, -1, 1);
      a = randVec(6, -1, 1);
      break;
    }
    default:
      break;
  }

  return std::make_tuple(q, v, a);
}


/** Exact integration of a constant velocity \a vel over a time interval of size \a step
  * starting from position \a q.
  */
std::vector<double> explicitIntegrationAtConstantSpeed(Joint::Type type,
                                                       double step,
                                                       const std::vector<double> & q,
                                                       const std::vector<double> & vel)
{
  switch(type)
  {
    case Joint::Rev:
    case Joint::Prism:
      return {q[0] + vel[0] * step};
    case Joint::Spherical:
    {
      Quaterniond qi(q[0], q[1], q[2], q[3]);
      Vector3d w(vel[0], vel[1], vel[2]);
      w *= step;
      double n = w.norm();
      double s = sva::sinc(n/2)/2;
      Quaterniond qexp(std::cos(n/2), s * w.x(), s * w.y(), s * w.z());
      Quaterniond qf = qi * qexp;
      return {qf.w(), qf.x(), qf.y(), qf.z()};
    }
    case Joint::Planar:
      break;
    case Joint::Cylindrical:
      return {q[0] + vel[0] * step, q[1] + vel[1] * step};
    case Joint::Free:
    {
      // integration of the orientation part
      Quaterniond qi(q[0], q[1], q[2], q[3]);
      Vector3d w(vel[0], vel[1], vel[2]);
      Vector3d tw = step*w;
      double n = w.norm();
      double tn = step * n;
      double s = sva::sinc(tn / 2) / 2;
      Quaterniond qexp(std::cos(tn / 2), s * tw.x(), s * tw.y(), s * tw.z());
      Quaterniond qf = qi * qexp;

      // integration of the position part: we need to integrate R(t)v
      // using Rodrigues formula, we have that the primitive of exp(t hat(w)) is
      // E(t) = (t I - cos(t ||w||)/||w||^2 hat(w) + (t - sin(t ||w||)/||w||^2)hat(w)^2)
      // We now compute xi + qi*(E(t)-E(0))*v
      Vector3d x(q[4], q[5], q[6]);
      Vector3d v(vel[3], vel[4], vel[5]);
      Vector3d wv = w.cross(v);
      Vector3d w2v = w.cross(wv);
      double a, b;
      if(tn > 1e-4)
        a = (1- std::cos(tn)) / (n * n);
      else
        a = tn * tn / 2;
      b = step * (1 - sva::sinc(tn)) / (n * n);
      Vector3d dx = step * v + a * wv + b * w2v;
      Vector3d xf = x + qi * dx;
      return {qf.w(), qf.x(), qf.y(), qf.z(), xf.x(), xf.y(), xf.z()};
    }
  }
}

void testConstantSpeedIntegration(Joint::Type type,
                                  double step,
                                  const std::vector<double> & q,
                                  const std::vector<double> & v)
{
  MultiBody mb;
  MultiBodyConfig mbc;
  MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeSingleJointRobot(type);

  mbc.q = {{}, q};
  mbc.alpha = {{}, v};
  mbc.alphaD = {{}, std::vector<double>(v.size(), 0)};
  forwardKinematics(mb, mbc);

  eulerIntegration(mb, mbc, step);

  auto q_expected2 = explicitIntegrationAtConstantSpeed(type, step, q, v);
  std::cout << "for type = " << type << ", with dt=" << step << std::endl;
  std::cout << "expected = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << q_expected2[i] << " ";
  std::cout << std::endl;
  std::cout << "computed = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << mbc.q[1][i] << " ";
  std::cout << std::endl;
  std::cout << "error    = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << std::abs(mbc.q[1][i] - q_expected2[i]) << " ";
  std::cout << std::endl;

  for(size_t i = 0; i < q.size(); ++i)
  {
    BOOST_CHECK_CLOSE_FRACTION(q_expected2[i], mbc.q[1][i], 1e-8);
  }
}

// This tests the consistency of the integration, i.e. that intergrating one time over
// step gives the same result as integrating N time over step/N
void testIntegrationConsistency(Joint::Type type,
                                double step,
                                const std::vector<double> & q,
                                const std::vector<double> & v,
                                const std::vector<double> & a)
{
  MultiBody mb;
  MultiBodyConfig mbc;
  MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeSingleJointRobot(type);
  MultiBodyConfig mbc0(mbc);

  mbc.q = {{}, q};
  mbc.alpha = {{}, v};
  mbc.alphaD = {{}, a};
  mbc0.q = {{}, q};
  mbc0.alpha = {{}, v};
  mbc0.alphaD = {{}, a};

  forwardKinematics(mb, mbc);
  forwardKinematics(mb, mbc0);

  // integrating on the whole time step
  eulerIntegration(mb, mbc0, step);

  // integrating with on small time step
  const int N = 2000;
  for(int i = 0; i < N; ++i)
  {
    eulerIntegration(mb, mbc, step / N);
    forwardKinematics(mb, mbc);
  }

  std::cout << "for type = " << type << ", with dt=" << step << std::endl;
  std::cout << "whole step  = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << mbc0.q[1][i] << " ";
  std::cout << std::endl;
  std::cout << "fractionned = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << mbc.q[1][i] << " ";
  std::cout << std::endl;
  std::cout << "error       = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << std::abs(mbc.q[1][i] - mbc0.q[1][i]) << " ";
  std::cout << std::endl;
  for(size_t i = 0; i < q.size(); ++i)
  {
    BOOST_CHECK_CLOSE_FRACTION(mbc0.q[1][i], mbc.q[1][i], 1e-4);
  }
}


void testConstantAccelerationIntegration(Joint::Type type,
                                         double step,
                                         const std::vector<double> & q,
                                         const std::vector<double> & v,
                                         const std::vector<double> & a)
{
  std::vector<double> qc = q;
  std::vector<double> vc = v;

  MultiBody mb;
  MultiBodyConfig mbc;
  MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeSingleJointRobot(type);

  mbc.q = {{}, q};
  mbc.alpha = {{}, v};
  mbc.alphaD = {{}, a};

  forwardKinematics(mb, mbc);

  // integrating on the whole time step
  eulerIntegration(mb, mbc, step);

  // integrating with constant velocity on small time step
  const int N = 10000;
  double dt = step / N;
  //we take the speed at the middle of the interval
  for(size_t i = 0; i < vc.size(); ++i) vc[i] = vc[i] + a[i] * dt/2;
  for(int i = 0; i < N; ++i)
  {
    qc = explicitIntegrationAtConstantSpeed(type, dt, qc, vc);
    for(size_t i = 0; i < vc.size(); ++i) vc[i] = vc[i] + a[i] * dt;
  }

  std::cout << "for type = " << type << ", with dt=" << step << std::endl;
  std::cout << "computed     = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << mbc.q[1][i] << " ";
  std::cout << std::endl;
  std::cout << "approximated = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << qc[i] << " ";
  std::cout << std::endl;
  std::cout << "error        = ";
  for(size_t i = 0; i < q.size(); ++i) std::cout << std::abs(mbc.q[1][i] - qc[i]) << " ";
  std::cout << std::endl;
  for(size_t i = 0; i < q.size(); ++i)
  {
    BOOST_CHECK_CLOSE_FRACTION(qc[i], mbc.q[1][i], 1e-4);
  }
}

void testConsistencyWithJacobian(Joint::Type type,
                                  const std::vector<double> & q,
                                  const std::vector<double> & v)
{
  double dt = 0.1;

  MultiBody mb;
  MultiBodyConfig mbc;
  MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeSingleJointRobot(type, Vector3d::UnitZ(), Eigen::Vector3d(1, 1, 1));

  Jacobian J(mb, "ee");

  mbc.q = {{}, q, {}};
  mbc.alpha = {{}, v, {}};
  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  // Compute via Simpson's rule an approximate integration of J(q)v over dt
  // and add it to the initial pose of the end effector
  const int N = 10;
  std::vector<double> q1;
  std::vector<double> q2;
  // Initial position of the end effector
  Vector3d pi = mbc.bodyPosW[2].translation();
  Vector3d vi = J.velocity(mb, mbc).linear();
  double delta = dt / N;
  for(int i = 0; i < N/2; ++i)
  {
    q1 = explicitIntegrationAtConstantSpeed(type, (2*i+1)*delta, q, v);
    mbc.q = {{}, q1, {}};
    forwardKinematics(mb, mbc);
    forwardVelocity(mb, mbc);
    Vector3d v1 = J.velocity(mb, mbc).linear();
    q2 = explicitIntegrationAtConstantSpeed(type, (2*i + 2)*delta, q, v);
    mbc.q = {{}, q2, {}};
    forwardKinematics(mb, mbc);
    forwardVelocity(mb, mbc);
    Vector3d v2 = J.velocity(mb, mbc).linear();
    pi += delta / 3 * (vi + 4 * v1 + v2);
    vi = v2;
  }

  // Compute the integration in one step of the joint velocity
  std::vector<double> qf = explicitIntegrationAtConstantSpeed(type, dt, q, v);
  mbc.q = {{}, qf, {}};
  forwardKinematics(mb, mbc);
  // Compute the position of the end effector at this configuration
  Vector3d pe = mbc.bodyPosW[2].translation();

  std::cout << "For type = " << type << std::endl;
  std::cout << "expected = " << pi.transpose() << std::endl;
  std::cout << "computed = " << pe.transpose() << std::endl;
  std::cout << "error    = " << (pi - pe).cwiseAbs().transpose() << std::endl;

  BOOST_CHECK_SMALL((pi - pe).norm(), 1e-10);
}

BOOST_AUTO_TEST_CASE(ConstantSpeedJointIntegrationTest)
{
  std::cout << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n" << std::endl;

  for(auto step : dt)
  {
    for(auto type : types)
    {
      std::vector<double> q, v, a;
      std::tie(q, v, a) = randQVA(type);
      testConstantSpeedIntegration(type, step, q, v);
    }
  }
}

BOOST_AUTO_TEST_CASE(IntegrationConsistencyTest)
{
  std::cout << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n" << std::endl;
  
  for (auto step : dt)
  {
    for (auto type : types)
    {
      std::vector<double> q, v, a;
      std::tie(q, v, a) = randQVA(type);
      testIntegrationConsistency(type, step, q, v, a);
    }
  }
}

BOOST_AUTO_TEST_CASE(ConstantAccelerationJointIntegrationTest)
{
  std::cout << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n" << std::endl;

  for(auto step : dt)
  {
    for(auto type : types)
    {
      std::vector<double> q, v, a;
      std::tie(q, v, a) = randQVA(type);
      testConstantAccelerationIntegration(type, step, q, v, a);
    }
  }
}

BOOST_AUTO_TEST_CASE(JacobianConsistencyTest)
{
  std::cout << "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n" << std::endl;

  for(auto type : types)
  {
    std::vector<double> q, v, a;
    std::tie(q, v, a) = randQVA(type);
    testConsistencyWithJacobian(type, q, v);
  }
}

BOOST_AUTO_TEST_CASE(JointIntegrationTest)
{
  const double pi = constants::pi<double>();
  const double c2 = std::sqrt(2) / 2;

  //testConstantSpeedIntegration(Joint::Rev, 1, {1}, {0.5}/*, {1.5}*/);
  //testConstantSpeedIntegration(Joint::Prism, 1, {1}, {0.5}/*, {1.5}*/);
  //testConstantSpeedIntegration(Joint::Spherical, 1, {1, 0, 0, 0}, {pi / 2, 0, 0} /*, {c2, c2, 0, 0}*/);
  //testConstantSpeedIntegration(Joint::Spherical, 1, {c2, 0, c2, 0}, {pi / 2, 0, 0}/*, {0.5, 0.5, 0.5, -0.5}*/);
  //// testConstantSpeedIntegration(Joint::Planar, 1, { 1 }, { 0.5 }/*, { 1.5 }*/);
  ////testConstantSpeedIntegration(Joint::Cylindrical, 1, {1, 2}, {0.5, 0.25}/*, {1.5, 2.25})*/;
  //testConstantSpeedIntegration(Joint::Free, 1, {1, 0, 0, 0, 0, 0, 0}, {pi / 2, 0, 0, 1, 0, 0}/*, {c2, c2, 0, 0, 1, 0, 0}*/);
  //testConstantSpeedIntegration(Joint::Free, 1, {1, 0, 0, 0, 0, 0, 0}, {0, pi / 2, 0, 0, 1, 0}/*, {c2, c2, 0, 0, 0, 1, 0}*/);
  //testConstantSpeedIntegration(Joint::Free, 1, {1, 0, 0, 0, 0, 0, 0}, {0, 0, pi / 2, 0, 0, 1}/*, {c2, c2, 0, 0, 0, 0, 1}*/);
  //testConstantSpeedIntegration(Joint::Free, .1, {1, 0, 0, 0, 1, 2, 3}, {pi / 2, 0, 0, 0.5, 0.25, -0.5}/*,
  //                             {c2, c2, 0, 0, 1.5, 2.25, 2.5}*/);
  //testConstantSpeedIntegration(Joint::Free, .1, {c2, 0, c2, 0, 1, 2, 3}, {pi / 2, 0, 0, 0.5, 0.25, -0.5}/*,
  //                             {0.5, 0.5, 0.5, -0.5, 0.5, 2.25, 2.5}*/);

  std::vector<double> q, v, a;
  //std::tie(q, v, a) = randQVA(Joint::Rev);
  //testConstantAccelerationIntegration(Joint::Rev, 1, q, v, a);
  //std::tie(q, v, a) = randQVA(Joint::Prism);
  //testConstantAccelerationIntegration(Joint::Prism, 1, q, v, a);
  //std::tie(q, v, a) = randQVA(Joint::Spherical);
  //testConstantAccelerationIntegration(Joint::Spherical, 0.1, q, v, a);
  //// std::tie(q, v, a) = randQVA(Joint::Planar);
  //// testConstantAccelerationIntegration(Joint::Planar, 1, q, v, a);
  //std::tie(q, v, a) = randQVA(Joint::Cylindrical);
  //testConstantAccelerationIntegration(Joint::Cylindrical, 1, q, v, a);
  //std::tie(q, v, a) = randQVA(Joint::Free);
  //testConstantAccelerationIntegration(Joint::Free, 0.01, q, v, a);
}
