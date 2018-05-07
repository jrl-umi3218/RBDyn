#include "Tree30Dof.h"

#define BOOST_TEST_MODULE Coriolis
#include <boost/test/unit_test.hpp>

#include <RBDyn/Coriolis.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

BOOST_AUTO_TEST_CASE(CoriolisTest)
{
	std::srand(133757348);

	constexpr double TOL = 1e-6;
	constexpr double BIGTOL = 50 * TOL;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;

	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::MultiBody mbt;
	rbd::MultiBodyConfig mbtc;
	rbd::MultiBodyGraph mbtg;

	std::tie(mbt, mbtc, mbtg) = makeTree30Dof(false);

	const static int ROUNDS = 1000;

	for (int i = 0; i < ROUNDS; ++i)
	{
		mbc.zero(mb);
		mbtc.zero(mbt);

		Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
		mbc.q = rbd::sVectorToParam(mb, q);

		for (auto &q : mbc.q)
		{
			if (q.size() == 7)
			{
				Eigen::Vector3d axis =
				    Eigen::Vector3d::Random();
				Eigen::AngleAxisd aa(0.5, axis / axis.norm());
				Eigen::Quaterniond qd(aa);
				q[0] = qd.w();
				q[1] = qd.x();
				q[2] = qd.y();
				q[3] = qd.z();
			}
		}

		mbtc.q = mbc.q;

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		rbd::forwardKinematics(mbt, mbtc);
		rbd::forwardVelocity(mbt, mbtc);

		rbd::ForwardDynamics fd(mbt);
		fd.computeC(mbt, mbtc);
		Eigen::VectorXd gravity = fd.C();

		Eigen::VectorXd qd = Eigen::VectorXd::Random(mb.nrDof());
		mbc.alpha = rbd::sVectorToDof(mb, qd);
		mbtc.alpha = mbc.alpha;

		rbd::forwardVelocity(mb, mbc);
		rbd::forwardVelocity(mbt, mbtc);

		rbd::Coriolis coriolis(mb);
		Eigen::MatrixXd C = coriolis.coriolis(mb, mbc);

		fd.computeC(mbt, mbtc);
		Eigen::MatrixXd N = fd.C();

		BOOST_CHECK_SMALL((C * qd + gravity - N).norm(), TOL);

		fd.computeH(mbt, mbtc);
		Eigen::MatrixXd m1 = fd.H();

		double dt = 1e-8;

		rbd::sEulerIntegration(mbt, mbtc, dt);

		rbd::forwardKinematics(mbt, mbtc);
		rbd::forwardVelocity(mbt, mbtc);
		fd.computeH(mbt, mbtc);

		Eigen::MatrixXd m2 = fd.H();

		Eigen::MatrixXd diff = (m2 - m1) / dt - (C + C.transpose());

		// Because we are using finite differences, the error is larger
		// on this test
		BOOST_CHECK_SMALL(diff.norm(), BIGTOL);
	}
}
