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
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"
#include "EulerIntegration.h"

const double TOL = 0.0000001;

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

	mbg.linkBodies(0, to, 1, from, 0);
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
		PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 0.5, 0)),
		PTransform(Vector3d(0., 1.5, 0.)), PTransform(Vector3d(0., 2.5, 0))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());


	// check rotX
	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 1.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 2.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].translation() - mbc.bodyPosW[i].translation()).norm(),
			TOL);
		BOOST_CHECK_SMALL((res[i].rotation() - mbc.bodyPosW[i].rotation()).norm(),
			TOL);
	}

	// check rotY
	mbc.q = {{}, {0.}, {cst::pi<double>()/2.}, {0.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(Vector3d(0., .5, 0.)),
		PTransform(RotY(cst::pi<double>()/2.), Vector3d(0., 1.5, 0.)),
		PTransform(RotY(cst::pi<double>()/2.), Vector3d(0., 2.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());


	// check rotZ
	mbc.q = {{}, {0.}, {0.}, {cst::pi<double>()/2.}};

	forwardKinematics(mb, mbc);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(Vector3d(0., 0.5, 0.)),
		PTransform(Vector3d(0., 1.5, 0.)),
		PTransform(RotZ(cst::pi<double>()/2.), Vector3d(0., 2.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());


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

	res = {PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 0.5, 0)),
		PTransform(Vector3d(0., 1.5, 0.)), PTransform(Vector3d(0., 2.5, 0)),
		PTransform(Vector3d(0.5, 1., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyPosW.begin(), mbc2.bodyPosW.end());
	// check sphere rot Y
	Quaterniond q(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitY()));
	mbc2.q = {{}, {0.}, {0.}, {0.}, {q.w(), q.x(), q.y(), q.z()}};

	forwardKinematics(mb2, mbc2);

	res = {PTransform(Vector3d(0., 0., 0.)), PTransform(Vector3d(0., 0.5, 0)),
		PTransform(Vector3d(0., 1.5, 0.)), PTransform(Vector3d(0., 2.5, 0)),
		PTransform(RotY(cst::pi<double>()/2.), Vector3d(0.5, 1., 0.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].translation() - mbc2.bodyPosW[i].translation()).norm(),
			TOL);
		BOOST_CHECK_SMALL((res[i].rotation() - mbc2.bodyPosW[i].rotation()).norm(),
			TOL);
	}


	// check j1 rotX
	mbc2.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};

	forwardKinematics(mb2, mbc2);

	res = {PTransform(Vector3d(0., 0., 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 0.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 1.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 2.)),
		PTransform(RotX(cst::pi<double>()/2.), Vector3d(0.5, 0.5, 0.5))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].translation() - mbc2.bodyPosW[i].translation()).norm(),
			TOL);
		BOOST_CHECK_SMALL((res[i].rotation() - mbc2.bodyPosW[i].rotation()).norm(),
			TOL);
	}

	// test safe version
	BOOST_CHECK_NO_THROW(sForwardKinematics(mb2, mbc2));

	// bad number of body
	MultiBodyConfig mbcBadNrBody = mbc2;
	mbcBadNrBody.bodyPosW.resize(4);

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



BOOST_AUTO_TEST_CASE(FVTest)
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

	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	//  Root     j0      j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	// check identity
	mbc.q = {{}, {0.}, {0.}, {0.}};
	mbc.alpha = {{}, {0.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	std::vector<MotionVec> res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot X
	mbc.alpha = {{}, {1.}, {0.}, {0.}};
	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., 0., 1.)),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., 0., 2.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot Y
	mbc.alpha = {{}, {0.}, {1.}, {0.}};
	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.)),
		MotionVec(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot Z
	mbc.alpha = {{}, {0.}, {0.}, {1.}};
	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot X with 90 X rotation
	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};
	mbc.alpha = {{}, {1.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., -1., 0.)),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., -2., 0.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
	}


	// check rot X with 90 Y rotation
	mbc.alpha = {{}, {0.}, {1.}, {0.}};

	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.)),
		MotionVec(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
	}



	// check rot X with 90 Z rotation
	mbc.alpha = {{}, {0.}, {0.}, {1.}};

	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., -1., 0.), Vector3d(0., 0., 0.))};





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
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	forwardKinematics(mb2, mbc2);
	forwardVelocity(mb2, mbc2);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());


	// check spherical X
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {1., 0., 0.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());



	// check spherical Y
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 1., 0.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());



	// check spherical Z
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 1.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()), MotionVec(Vector6d::Zero()),
		MotionVec(Vector6d::Zero()),
		MotionVec(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());
}



BOOST_AUTO_TEST_CASE(FreeFlyerTest)
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

	mbg.addBody(b0);

	MultiBody mb = mbg.makeMultiBody(0, false);

	MultiBodyConfig mbc(mb);

	// check identity
	mbc.q = {{1., 0., 0., 0., 0., 0., 0.}};
	mbc.alpha = {{0., 0., 0., 0., 0., 0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	std::vector<MotionVec> res = {MotionVec(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());


	// check Y Rot
	Quaterniond q = Quaterniond::Identity();
	mbc.q = {{q.w(), q.x(), q.y(), q.z(), 1., 0., 0.}};
	mbc.alpha = {{0., 1., 0., 0., 0., 0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	res = {MotionVec(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());
}



BOOST_AUTO_TEST_CASE(EulerTest)
{
	using namespace std;
	using namespace Eigen;
	using namespace rbd;

	// 1 dof joint

	// static
	vector<double> q = {0.};

	eulerJointIntegration(Joint::RevX, {0.}, 1., q);

	BOOST_CHECK_EQUAL(q[0], 0.);

	// moving
	eulerJointIntegration(Joint::RevX, {1.}, 1., q);

	BOOST_CHECK_EQUAL(q[0], 1.);


	// free

	// static
	q = {1., 0., 0., 0., 0., 0., 0.};
	vector<double> goalQ = q;
	eulerJointIntegration(Joint::Spherical, {0., 0., 0., 0., 0., 0.}, 1., q);
	BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());

	// X unit move
	goalQ = {1., 0., 0., 0., 1., 0., 0.},
	eulerJointIntegration(Joint::Free, {0., 0., 0., 1., 0., 0.}, 1., q);
	BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());


	// X unit rot
	q = {1., 0., 0., 0., 0., 0., 0.};
	goalQ = {1./std::sqrt(1.25), 0.5/std::sqrt(1.25), 0., 0., 0., 0., 0.},
	eulerJointIntegration(Joint::Free, {1., 0., 0., 0., 0., 0.}, 1., q);
	BOOST_CHECK_EQUAL_COLLECTIONS(q.begin(), q.end(), goalQ.begin(), goalQ.end());
}

