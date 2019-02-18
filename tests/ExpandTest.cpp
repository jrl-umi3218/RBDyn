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

#include <RBDyn/Jacobian.h>

#define BOOST_TEST_MODULE Expand
#include <boost/test/unit_test.hpp>

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

BOOST_AUTO_TEST_CASE(ExpandJacobianTest)
{
	std::srand(133757348);

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;

	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	const static int ROUNDS = 1000;

	rbd::Jacobian jac(mb, mb.body(mb.nrBodies() - 1).name());
	rbd::Blocks compact = jac.compactPath(mb);

	for(int i = 0; i < ROUNDS; ++i)
	{
		mbc.zero(mb);

		Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
		mbc.q = rbd::vectorToParam(mb, q);
		setRandomFreeFlyer(mbc);

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		Eigen::MatrixXd jacMat = jac.jacobian(mb, mbc);

		Eigen::MatrixXd fullJacMat(6, mb.nrDof());
		jac.fullJacobian(mb, jacMat, fullJacMat);

		Eigen::MatrixXd res = fullJacMat.transpose()*fullJacMat;

		Eigen::MatrixXd product = jacMat.transpose()*jacMat;
		Eigen::MatrixXd fullProduct = jac.expand(mb, product);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

		fullProduct.setZero();
		jac.expandAdd(mb, product, fullProduct);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

		fullProduct.setZero();
		jac.expandAdd(compact, product, fullProduct);

		BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);
	}
}
