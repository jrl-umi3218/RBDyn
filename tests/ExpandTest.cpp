#include "Tree30Dof.h"

#include <RBDyn/Coriolis.h>

#define BOOST_TEST_MODULE Expand
#include <boost/test/unit_test.hpp>

#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/EulerIntegration.h>

#include <iostream>

BOOST_AUTO_TEST_CASE(ExpandJacobianTest)
{
	std::srand(133757348);

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;

	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	const static int ROUNDS = 1000;

	rbd::Jacobian jac(mb, mb.body(mb.nrBodies() - 1).name());
	rbd::Blocks compact = rbd::compactPath(jac, mb);

	for(int i = 0; i < ROUNDS; ++i)
	{
		mbc.zero(mb);

		Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
		mbc.q = rbd::sVectorToParam(mb, q);

		for(auto& q : mbc.q)
		{
			if(q.size() == 7)
			{
				Eigen::Vector3d axis = Eigen::Vector3d::Random();
				Eigen::AngleAxisd aa(0.5, axis/axis.norm());
				Eigen::Quaterniond qd(aa);
				q[0] = qd.w();
				q[1] = qd.x();
				q[2] = qd.y();
				q[3] = qd.z();
			}
		}

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		Eigen::MatrixXd jacMat = jac.jacobian(mb, mbc);

		Eigen::MatrixXd fullJacMat(6, mb.nrDof());
		jac.fullJacobian(mb, jacMat, fullJacMat);

		Eigen::MatrixXd res = fullJacMat.transpose()*fullJacMat;

		Eigen::MatrixXd product = jacMat.transpose()*jacMat;
		Eigen::MatrixXd fullProduct = rbd::expand(jac, mb, product);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

		fullProduct.setZero();
		rbd::expandAdd(jac, mb, product, fullProduct);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

		fullProduct.setZero();
		rbd::compactExpandAdd(compact, product, fullProduct);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);
	}
}
