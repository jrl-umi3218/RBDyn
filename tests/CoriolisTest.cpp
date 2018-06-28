// Copyright 2012-2018 CNRS-UM LIRMM, CNRS-AIST JRL
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

#include "Tree30Dof.h"

#define BOOST_TEST_MODULE Coriolis
#include <boost/test/unit_test.hpp>

#include <RBDyn/Coriolis.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>


void setRandomFreeFlyer(rbd::MultiBodyConfig& mbc)
{
	Eigen::Vector3d axis =
	    Eigen::Vector3d::Random();
	Eigen::AngleAxisd aa(0.5, axis / axis.norm());
	Eigen::Quaterniond qd(aa);
	mbc.q[0][0] = qd.w();
	mbc.q[0][1] = qd.x();
	mbc.q[0][2] = qd.y();
	mbc.q[0][3] = qd.z();
}

BOOST_AUTO_TEST_CASE(CoriolisTest)
{
	std::srand(133757348);

	constexpr double TOL = 1e-6;
	constexpr double BIGTOL = 50 * TOL;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;

	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::ForwardDynamics fd(mb);
	rbd::Coriolis coriolis(mb);

	const static int ROUNDS = 100;

	for (int i = 0; i < ROUNDS; ++i)
	{
		mbc.zero(mb);

		Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
		mbc.q = rbd::vectorToParam(mb, q);
		setRandomFreeFlyer(mbc);

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		fd.computeC(mb, mbc);
		Eigen::VectorXd gravity = fd.C();

		Eigen::VectorXd qd = Eigen::VectorXd::Random(mb.nrDof());
		mbc.alpha = rbd::vectorToDof(mb, qd);

		rbd::forwardVelocity(mb, mbc);

		Eigen::MatrixXd C = coriolis.coriolis(mb, mbc);

		fd.computeC(mb, mbc);
		Eigen::MatrixXd N = fd.C();

		BOOST_CHECK_SMALL((C * qd + gravity - N).norm(), TOL);

		fd.computeH(mb, mbc);
		Eigen::MatrixXd m1 = fd.H();

		double dt = 1e-8;

		rbd::eulerIntegration(mb, mbc, dt);

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);
		fd.computeH(mb, mbc);

		Eigen::MatrixXd m2 = fd.H();

		Eigen::MatrixXd diff = (m2 - m1) / dt - (C + C.transpose());

		// Because we are using finite differences, the error is larger
		// on this test
		BOOST_CHECK_SMALL(diff.norm(), BIGTOL);
	}
}
