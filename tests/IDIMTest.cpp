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
// std
#include <iostream>

// boost
#define BOOST_TEST_MODULE IDIMTest
#include <boost/test/unit_test.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/ID.h"
#include "RBDyn/IDIM.h"

// arm
#include "Tree30Dof.h"


sva::RBInertiad randomInertia()
{
	Eigen::Matrix<double, 10, 1> v(Eigen::Matrix<double, 10, 1>::Random());
	v(0) = std::abs(v(0)) + 0.001; // positive mass
	v.tail<6>() = v.tail<6>().array().abs(); // positive inertia
	return rbd::vectorToInertia(v);
}


BOOST_AUTO_TEST_CASE(toFrom10Vec)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double mass = 11.11;
	Matrix3d I;
	I << 1., 2., 3.,
			 2., 10., 4.,
			 3., 4., 12.;
	Vector3d h = Vector3d::Random()*100.;

	RBInertiad rbi(mass, h, I);
	auto vec10d = inertiaToVector(rbi);
	RBInertiad rbi2 = vectorToInertia(vec10d);

	BOOST_CHECK_EQUAL(rbi, rbi2);
}


BOOST_AUTO_TEST_CASE(IMPhiTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	// test that Im = IMPhi(m)*phi_i
	for(int i = 0; i < 100; ++i)
	{
		RBInertiad rbi(randomInertia());
		MotionVecd m(Vector6d::Random());

		ForceVecd res1 = rbi*m;
		ForceVecd res2(IMPhi(m)*inertiaToVector(rbi));

		BOOST_CHECK_SMALL((res1 - res2).vector().norm(), 1e-8);
	}

	// test that m x^* Im = m x^* IMPhi(m)*phi_i
	for(int i = 0; i < 100; ++i)
	{
		RBInertiad rbi(randomInertia());
		MotionVecd m(Vector6d::Random());

		ForceVecd res1 = m.crossDual(rbi*m);
		ForceVecd res2((vector6ToCrossDualMatrix(m.vector())*IMPhi(m))*inertiaToVector(rbi));

		BOOST_CHECK_SMALL((res1 - res2).vector().norm(), 1e-8);
	}
}


BOOST_AUTO_TEST_CASE(computeY)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	// first we initialize all the inertia of our multibody with random data
	std::vector<Body> oldB(mb.bodies());
	std::vector<Body> newB;
	newB.reserve(oldB.size());
	for(const Body& b: mb.bodies())
	{
		newB.push_back(Body(randomInertia(), b.name()));
	}
	mb = MultiBody(newB, mb.joints(), mb.predecessors(), mb.successors(),
		mb.parents(), mb.transforms());

	VectorXd inertiaVec(multiBodyToInertialVector(mb));
	VectorXd idimTorque(mb.nrDof());
	VectorXd idTorque(mb.nrDof());

	InverseDynamics id(mb);
	IDIM idim(mb);

	for(int i = 0; i < 50; ++i)
	{
		VectorXd q(VectorXd::Random(mb.nrParams())*4.);
		VectorXd alpha(VectorXd::Random(mb.nrDof()));
		VectorXd alphaD(VectorXd::Random(mb.nrDof()));
		vectorToParam(q, mbc.q);
		vectorToParam(alpha, mbc.alpha);
		vectorToParam(alphaD, mbc.alphaD);

		forwardKinematics(mb, mbc);
		forwardVelocity(mb, mbc);

		id.inverseDynamics(mb, mbc);
		idim.sComputeY(mb, mbc);

		paramToVector(mbc.jointTorque, idTorque);
		idimTorque = idim.Y()*inertiaVec;

		BOOST_CHECK_SMALL((idTorque - idimTorque).norm(), 1e-8);
	}
}
