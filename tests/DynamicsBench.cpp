/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// benchmark
#include "benchmark/benchmark.h"

// RBDyn
#include "RBDyn/CoM.h"
#include "RBDyn/Coriolis.h"
#include "RBDyn/FD.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"

// Arm
#include "Tree30Dof.h"

static void BM_FD_computeH(benchmark::State & state)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::ForwardDynamics fd(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);
	for(auto _ : state)
	{
		fd.computeH(mb, mbc);
	}
}
BENCHMARK(BM_FD_computeH);

static void BM_FD_computeC(benchmark::State & state)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::ForwardDynamics fd(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);
	for(auto _ : state)
	{
		fd.computeC(mb, mbc);
	}
}
BENCHMARK(BM_FD_computeC);

static void BM_Coriolis(benchmark::State & state)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	rbd::Coriolis coriolis(mb);

	for(auto _ : state)
	{
		coriolis.coriolis(mb, mbc);
	}
}
BENCHMARK(BM_Coriolis);

BENCHMARK_MAIN()
