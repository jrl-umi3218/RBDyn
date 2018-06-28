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
