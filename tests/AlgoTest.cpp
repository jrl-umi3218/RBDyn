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
#define BOOST_TEST_MODULE FKTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg>

// RBDyn
#include "FK.h"
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"

BOOST_AUTO_TEST_CASE(FKTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertia rbi(mass, h, I);

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(rbi, 2, "b2");
	Body b3(rbi, 3, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::RevX, true, 0, "j0");
	Joint j1(Joint::RevY, true, 1, "j1");
	Joint j2(Joint::RevZ, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));

	mbg.linkBodies(0, PTransform(Vector3d(0., 1., 0.)), 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	//  Root     j0      j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	// check identity
	mbc.q = {{}, {0.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);

	std::vector<PTransform> res = {
		PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 1., 0)),
		PTransform(Vector3d(0., 2., 0.)), PTransform(Vector3d(0., 3., 0))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyGlobal.begin(), mbc.bodyGlobal.end());


	// check rotX
	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 1.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 2.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyGlobal.begin(), mbc.bodyGlobal.end());


	// check rotY
	mbc.q = {{}, {0.}, {cst::pi<double>()/2.}, {0.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(Vector3d(0., 1., 0.)),
		PTransform(RotY(cst::pi<double>()/2.), Vector3d(0., 2., 0.)),
		PTransform(RotY(cst::pi<double>()/2.), Vector3d(0., 3., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyGlobal.begin(), mbc.bodyGlobal.end());


	// check rotZ
	mbc.q = {{}, {0.}, {0.}, {cst::pi<double>()/2.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(Vector3d(0., 1., 0.)),
		PTransform(Vector3d(0., 2., 0.)),
		PTransform(RotZ(cst::pi<double>()/2.), Vector3d(0., 3., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyGlobal.begin(), mbc.bodyGlobal.end());

	//                b4
	//             j3 | Spherical
	//  Root     j0   |   j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ

	Body b4(rbi, 4, "b4");
	Joint j3(Joint::Spherical, true, 3, "j3");

	mbg.addBody(b4);
	mbg.addJoint(j3);

	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 4, PTransform(Vector3d(-0.5, 0., 0.)), 3);

	MultiBody mb2 = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc2(mb2);


	// check identity
	mbc2.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};

	forwardKinematics(mb2, mbc2);

	res = {PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 1., 0)),
		PTransform(Vector3d(0., 2., 0.)), PTransform(Vector3d(0., 3., 0)),
		PTransform(Vector3d(0.5, 1.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyGlobal.begin(), mbc2.bodyGlobal.end());

	// check sphere rot Y
	Quaterniond q(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitY()));
	mbc2.q = {{}, {0.}, {0.}, {0.}, {q.w(), q.x(), q.y(), q.z()}};

	forwardKinematics(mb2, mbc2);

	res = {PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 1., 0)),
		PTransform(Vector3d(0., 2., 0.)), PTransform(Vector3d(0., 3., 0)),
		// PTransform(RotY(cst::pi<double>()/2.), Vector3d(0.5, 1.5, 0.))}; lake of precision
		PTransform(q.inverse().matrix(), Vector3d(0.5, 1.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyGlobal.begin(), mbc2.bodyGlobal.end());

	// check j1 rotX
	mbc2.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};

	forwardKinematics(mb2, mbc2);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 1.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 1., 2.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0.5, 1., 0.5))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyGlobal.begin(), mbc2.bodyGlobal.end());


	// test safe version
	BOOST_CHECK_NO_THROW(sForwardKinematics(mb2, mbc2));

	// bad number of body
	MultiBodyConfig mbcBadNrBody = mbc2;
	mbcBadNrBody.bodyGlobal.resize(4);

	BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadNrBody), std::domain_error);

	// bad number of generalized position variable
	MultiBodyConfig mbcBadNrQ = mbc2;
	mbcBadNrQ.q = {{0.}, {0.}, {0.}, {1., 0., 0., 0.}};

	BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadNrQ), std::domain_error);

	// bad generalized position variable size
	MultiBodyConfig mbcBadQSize = mbc2;
	mbcBadQSize.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0.}};

	BOOST_CHECK_THROW(sForwardKinematics(mb2, mbcBadQSize), std::domain_error);
}
