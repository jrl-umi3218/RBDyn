/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// benchmark
#include "benchmark/benchmark.h"

// RBDyn
#include "RBDyn/CoM.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/Momentum.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"

// Arm
#include "Tree30Dof.h"

static void BM_Jacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::Jacobian jac(mb, "LARM6");

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobian(mb, mbc);
  }
}
BENCHMARK(BM_Jacobian);

static void BM_BodyJacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::Jacobian jac(mb, "LARM6");

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.bodyJacobian(mb, mbc);
  }
}
BENCHMARK(BM_BodyJacobian);

static void BM_VectorBodyJacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::Jacobian jac(mb, "LARM6");

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.vectorBodyJacobian(mb, mbc, Eigen::Vector3d(1., 0., 0.));
  }
}
BENCHMARK(BM_VectorBodyJacobian);

static void BM_JacobianDot(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::Jacobian jac(mb, "LARM6");

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobianDot(mb, mbc);
  }
}
BENCHMARK(BM_JacobianDot);

static void BM_BodyJacobianDot(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::Jacobian jac(mb, "LARM6");

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.bodyJacobianDot(mb, mbc);
  }
}
BENCHMARK(BM_BodyJacobianDot);

static void BM_CoMJacobianDummy_jacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CoMJacobianDummy jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobian(mb, mbc);
  }
}
BENCHMARK(BM_CoMJacobianDummy_jacobian);

static void BM_CoMJacobianDummy_jacobianDot(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CoMJacobianDummy jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobianDot(mb, mbc);
  }
}
BENCHMARK(BM_CoMJacobianDummy_jacobianDot);

static void BM_CoMJacobian_jacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CoMJacobian jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobian(mb, mbc);
  }
}
BENCHMARK(BM_CoMJacobian_jacobian);

static void BM_CoMJacobian_jacobianDot(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CoMJacobian jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.jacobianDot(mb, mbc);
  }
}
BENCHMARK(BM_CoMJacobian_jacobianDot);

static void BM_MomentumJacobian_jacobian(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CentroidalMomentumMatrix jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  Eigen::Vector3d com = rbd::computeCoM(mb, mbc);

  for(auto _ : state)
  {
    jac.computeMatrix(mb, mbc, com);
  }
}
BENCHMARK(BM_MomentumJacobian_jacobian);

static void BM_MomentumJacobian_jacobianDot(benchmark::State & state)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  rbd::CentroidalMomentumMatrix jac(mb);

  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  Eigen::Vector3d com = rbd::computeCoM(mb, mbc);
  Eigen::Vector3d comDot = rbd::computeCoMVelocity(mb, mbc);

  for(auto _ : state)
  {
    jac.computeMatrixDot(mb, mbc, com, comDot);
  }
}
BENCHMARK(BM_MomentumJacobian_jacobianDot);

BENCHMARK_MAIN()
