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
#define BOOST_TEST_MODULE Algo
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg>

// RBDyn
#include "FK.h"
#include "FV.h"
#include "ID.h"
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"

const double TOL = 0.0000001;


BOOST_AUTO_TEST_CASE(OneBody)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d(0., 0.5, 0.);

	RBInertia rbi(mass, h, I);

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");

	Joint j0(Joint::RevX, true, 0, "j0");

	MultiBodyGraph mbg;

	mbg.addBody(b0);
	mbg.addBody(b1);

	mbg.addJoint(j0);

	mbg.linkBodies(0, PTransform::Identity(),
								 1, PTransform(Vector3d(0., -0.5, 0.)), 0);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);
	mbc.q = {{}, {0.}};
	mbc.alpha = {{}, {0.}};
	mbc.alphaD = {{}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	InverseDynamics id(mb);
	id.inverseDynamics(mb, mbc);

	BOOST_CHECK_SMALL(mbc.jointTorque[1][0], TOL);

	mbc.q = {{}, {cst::pi<double>()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);
	id.inverseDynamics(mb, mbc);

	std::cout << std::endl;
	BOOST_CHECK_SMALL(mbc.jointTorque[1][0], TOL);

	mbc.q = {{}, {cst::pi<double>()/2.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);
	id.inverseDynamics(mb, mbc);

	double torque = Vector3d(0., 0., 0.5).cross(Vector3d(0., -9.81, 0.))(0);
	BOOST_CHECK_SMALL(std::abs(torque - mbc.jointTorque[1][0]), TOL);

	MultiBodyConfig mbc2 = mbc;
	mbc2.gravity = Vector3d::Zero();
	ForceVec gravity(Vector3d::Zero(), Vector3d(0., 9.81, 0.));
	ForceVec gravityB1 = mbc2.bodyPosW[0].dualMul(gravity);
	ForceVec gravityB2 = PTransform(Vector3d(0., 0., -0.5)).dualMul(gravity);
	mbc2.force = {gravityB1,
								gravityB2};
	id.inverseDynamics(mb, mbc2);

	torque = Vector3d(0., 0., 0.5).cross(Vector3d(0., -9.81, 0.))(0);
	BOOST_CHECK_SMALL(std::abs(torque - mbc2.jointTorque[1][0]), TOL);
}

