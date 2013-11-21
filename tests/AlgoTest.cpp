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
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Algo
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"
#include "EulerIntegration.h"
#include "Jacobian.h"

// arm
#include "XYZarm.h"
#include "XYZSarm.h"


const double TOL = 0.0000001;

BOOST_AUTO_TEST_CASE(FKTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	rbd::MultiBody mb, mb2;
	rbd::MultiBodyConfig mbc, mbc2;
	std::tie(mb, mbc) = makeXYZarm();
	std::tie(mb2, mbc2) = makeXYZSarm();

	// check identity
	mbc.q = {{}, {0.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);

	std::vector<PTransformd> res = {
		PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)),
		PTransformd(Vector3d(0., 1.5, 0.)), PTransformd(Vector3d(0., 2.5, 0))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());


	// check rotX
	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);

	res = {PTransformd(Vector3d(0., 0., 0.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 0.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 1.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 2.))};

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

	res = {PTransformd(Vector3d(0., 0., 0.)),
		PTransformd(Vector3d(0., .5, 0.)),
		PTransformd(RotY(cst::pi<double>()/2.), Vector3d(0., 1.5, 0.)),
		PTransformd(RotY(cst::pi<double>()/2.), Vector3d(0., 2.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());


	// check rotZ
	mbc.q = {{}, {0.}, {0.}, {cst::pi<double>()/2.}};

	forwardKinematics(mb, mbc);

	res = {PTransformd(Vector3d(0., 0., 0.)),
		PTransformd(Vector3d(0., 0.5, 0.)),
		PTransformd(Vector3d(0., 1.5, 0.)),
		PTransformd(RotZ(cst::pi<double>()/2.), Vector3d(0., 2.5, 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyPosW.begin(), mbc.bodyPosW.end());



	// check identity
	mbc2.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};

	forwardKinematics(mb2, mbc2);

	res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)),
		PTransformd(Vector3d(0., 1.5, 0.)), PTransformd(Vector3d(0., 2.5, 0)),
		PTransformd(Vector3d(0.5, 1., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyPosW.begin(), mbc2.bodyPosW.end());
	// check sphere rot Y
	Quaterniond q(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitY()));
	mbc2.q = {{}, {0.}, {0.}, {0.}, {q.w(), q.x(), q.y(), q.z()}};

	forwardKinematics(mb2, mbc2);

	res = {PTransformd(Vector3d(0., 0., 0.)), PTransformd(Vector3d(0., 0.5, 0)),
		PTransformd(Vector3d(0., 1.5, 0.)), PTransformd(Vector3d(0., 2.5, 0)),
		PTransformd(RotY(cst::pi<double>()/2.), Vector3d(0.5, 1., 0.))};

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

	res = {PTransformd(Vector3d(0., 0., 0.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 0.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 1.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0., 0.5, 2.)),
		PTransformd(RotX(cst::pi<double>()/2.), Vector3d(0.5, 0.5, 0.5))};

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

	rbd::MultiBody mb, mb2;
	rbd::MultiBodyConfig mbc, mbc2;
	std::tie(mb, mbc) = makeXYZarm();
	std::tie(mb2, mbc2) = makeXYZSarm();

	// check identity
	mbc.q = {{}, {0.}, {0.}, {0.}};
	mbc.alpha = {{}, {0.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	std::vector<MotionVecd> res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot X
	mbc.alpha = {{}, {1.}, {0.}, {0.}};
	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 1.)),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 2.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot Y
	mbc.alpha = {{}, {0.}, {1.}, {0.}};
	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.)),
		MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot Z
	mbc.alpha = {{}, {0.}, {0.}, {1.}};
	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());



	// check rot X with 90 X rotation
	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};
	mbc.alpha = {{}, {1.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.)),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., -1., 0.)),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., -2., 0.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
	}


	// check rot X with 90 Y rotation
	mbc.alpha = {{}, {0.}, {1.}, {0.}};

	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.)),
		MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

	for(size_t i = 0; i < res.size(); ++i)
	{
		BOOST_CHECK_SMALL((res[i].vector() - mbc.bodyVelW[i].vector()).norm(), TOL);
	}



	// check rot X with 90 Z rotation
	mbc.alpha = {{}, {0.}, {0.}, {1.}};

	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., -1., 0.), Vector3d(0., 0., 0.))};



	// check identity
	mbc2.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	forwardKinematics(mb2, mbc2);
	forwardVelocity(mb2, mbc2);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());


	// check spherical X
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {1., 0., 0.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(1., 0., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());



	// check spherical Y
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 1., 0.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc2.bodyVelW.begin(), mbc2.bodyVelW.end());



	// check spherical Z
	mbc2.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 1.}};

	forwardVelocity(mb2, mbc2);

	res = {MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()), MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector6d::Zero()),
		MotionVecd(Vector3d(0., 0., 1.), Vector3d(0., 0., 0.))};

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

	RBInertiad rbi(mass, h, I);

	Body b0(rbi, 0, "b0");

	mbg.addBody(b0);

	MultiBody mb = mbg.makeMultiBody(0, false);

	MultiBodyConfig mbc(mb);

	// check identity
	mbc.q = {{1., 0., 0., 0., 0., 0., 0.}};
	mbc.alpha = {{0., 0., 0., 0., 0., 0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	std::vector<MotionVecd> res = {MotionVecd(Vector6d::Zero())};

	BOOST_CHECK_EQUAL_COLLECTIONS(res.begin(), res.end(),
		mbc.bodyVelW.begin(), mbc.bodyVelW.end());


	// check Y Rot
	Quaterniond q = Quaterniond::Identity();
	mbc.q = {{q.w(), q.x(), q.y(), q.z(), 1., 0., 0.}};
	mbc.alpha = {{0., 1., 0., 0., 0., 0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	res = {MotionVecd(Vector3d(0., 1., 0.), Vector3d(0., 0., 0.))};

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

	eulerJointIntegration(Joint::Rev, {0.}, 1., q);

	BOOST_CHECK_EQUAL(q[0], 0.);

	// moving
	eulerJointIntegration(Joint::Rev, {1.}, 1., q);

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



BOOST_AUTO_TEST_CASE(FVInteg)
{
	using namespace std;
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double step = 1e-5;

	Body b = Body(1., Vector3d(0.1, 0.1, 0.1), Matrix3d::Identity(), 1, "body");
	MultiBodyGraph mbg;
	mbg.addBody(b);
	MultiBody mb = mbg.makeMultiBody(1, false);
	MultiBodyConfig mbc(mb);

	mbc.q = {{1., 0., 0., 0., 0., 0., 0.}};
	mbc.alpha = {{1., 0., 0., 0., 0., 1.}};
	mbc.alphaD = {{0., 0., 0., 0., 0., 0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	PTransformd oldPt = mbc.bodyPosW[0];

	for(int i = 0; i < 1000; ++i)
	{
		MotionVecd mvFV = mbc.bodyVelW[0];

		eulerIntegration(mb, mbc, step);
		forwardKinematics(mb, mbc);
		forwardVelocity(mb, mbc);

		PTransformd pt = mbc.bodyPosW[0];

		Vector3d mvDiff((pt.translation() - oldPt.translation())/step);

		std::cout << (mvDiff - mvFV.linear()).norm() << std::endl;

		oldPt = pt;
	}
}


BOOST_AUTO_TEST_CASE(FATest)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	std::tie(mb, mbc) = makeXYZSarm();

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);
	forwardAcceleration(mb, mbc);

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		BOOST_CHECK_SMALL(mbc.bodyAccB[i].vector().norm(), TOL);
	}

	std::vector<rbd::Jacobian> jacs(mb.nrBodies());
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacs[i] = rbd::Jacobian(mb, i);
	}

	Eigen::MatrixXd fullJac(6, mb.nrDof());
	Eigen::MatrixXd fullJacDot(6, mb.nrDof());

	for(int i = 0; i < 10; ++i)
	{
		Eigen::VectorXd q(mb.nrParams()), alpha(mb.nrDof()), alphaD(mb.nrDof());
		q.setRandom();
		q.tail<4>().normalize();
		alpha.setRandom();
		alphaD.setRandom();

		rbd::vectorToParam(q, mbc.q);
		rbd::vectorToParam(alpha, mbc.alpha);
		rbd::vectorToParam(alphaD, mbc.alphaD);

		forwardKinematics(mb, mbc);
		forwardVelocity(mb, mbc);
		forwardAcceleration(mb, mbc);

		for(int j = 0; j < mb.nrBodies(); ++j)
		{
			const Eigen::MatrixXd& jac = jacs[j].bodyJacobian(mb, mbc);
			const Eigen::MatrixXd& jacDot = jacs[j].bodyJacobianDot(mb, mbc);

			jacs[j].fullJacobian(mb, jac, fullJac);
			jacs[j].fullJacobian(mb, jacDot, fullJacDot);

			Eigen::Vector6d acc = fullJac*alphaD + fullJacDot*alpha;
			BOOST_CHECK_SMALL((mbc.bodyAccB[j].vector() - acc).norm(), TOL);
		}
	}
}

