// Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
//
// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

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
