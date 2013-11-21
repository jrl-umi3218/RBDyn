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
#define BOOST_TEST_MODULE BodyTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "EulerIntegration.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"
#include "CoM.h"

const double TOL = 0.000001;

BOOST_AUTO_TEST_CASE(computeCoMTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();
	Vector6d v = Vector6d::Zero();

	RBInertiad rbi(mass, h, I);

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(RBInertiad(2., h, I), 2, "b2");
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


	//  Root     j0      j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ

	PTransformd to(Vector3d(0., 0.5, 0.));
	PTransformd from(Vector3d(0., 0., 0.));

	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);


	MultiBody mb = mbg.makeMultiBody(0, true);
	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	mbc.q = {{}, {0.}, {0.}, {0.}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);
	forwardAcceleration(mb, mbc);

	Vector3d CoM = computeCoM(mb, mbc);
	Vector3d CoMV = computeCoMVelocity(mb, mbc);
	Vector3d CoMA = computeCoMAcceleration(mb, mbc);

	double handCoMX = 0.;
	double handCoMY = (0.5*1. + 1.*2. + 1.5*1.)/5.;
	double handCoMZ = 0.;
	BOOST_CHECK_EQUAL(CoM, Vector3d(handCoMX, handCoMY, handCoMZ));
	BOOST_CHECK_EQUAL(CoMV, Vector3d::Zero());
	BOOST_CHECK_EQUAL(CoMA, Vector3d::Zero());



	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);
	forwardAcceleration(mb, mbc);

	CoM = sComputeCoM(mb, mbc);
	CoMV = sComputeCoMVelocity(mb, mbc);
	CoMA = computeCoMAcceleration(mb, mbc);

	handCoMX = 0.;
	handCoMY = (0.5*1. + 0.5*2 + 0.5*1.)/5.;
	handCoMZ = (0.5*2. + 1.*1.)/5.;

	BOOST_CHECK_EQUAL(CoM, Vector3d(handCoMX, handCoMY, handCoMZ));
	BOOST_CHECK_EQUAL(CoMV, Vector3d::Zero());
	BOOST_CHECK_EQUAL(CoMA, Vector3d::Zero());


	// test safe version
	mbc.bodyPosW = {I, I, I};
	BOOST_CHECK_THROW(sComputeCoM(mb, mbc), std::domain_error);
	BOOST_CHECK_THROW(sComputeCoMVelocity(mb, mbc), std::domain_error);
	BOOST_CHECK_THROW(sComputeCoMAcceleration(mb, mbc), std::domain_error);

	mbc.bodyPosW = {I, I, I, I};
	mbc.bodyVelB = {v, v, v};
	mbc.bodyAccB = {v, v, v};
	BOOST_CHECK_NO_THROW(sComputeCoM(mb, mbc));
	BOOST_CHECK_THROW(sComputeCoMVelocity(mb, mbc), std::domain_error);
	BOOST_CHECK_THROW(sComputeCoMAcceleration(mb, mbc), std::domain_error);
}


Eigen::Vector3d makeCoMDotFromStep(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc)
{
	using namespace Eigen;
	using namespace rbd;

	double step = 1e-8;

	MultiBodyConfig mbcTmp(mbc);

	Vector3d oC = computeCoM(mb, mbcTmp);
	eulerIntegration(mb, mbcTmp, step);
	forwardKinematics(mb, mbcTmp);
	forwardVelocity(mb, mbcTmp);
	Vector3d nC = computeCoM(mb, mbcTmp);

	return (nC - oC)/step;
}


Eigen::MatrixXd makeJDotFromStep(const rbd::MultiBody& mb,
													const rbd::MultiBodyConfig& mbc,
													rbd::CoMJacobianDummy& jac)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double step = 1e-8;

	MultiBodyConfig mbcTmp(mbc);

	MatrixXd oJ = jac.jacobian(mb, mbcTmp);
	eulerIntegration(mb, mbcTmp, step);
	forwardKinematics(mb, mbcTmp);
	forwardVelocity(mb, mbcTmp);
	MatrixXd nJ = jac.jacobian(mb, mbcTmp);

	return (nJ - oJ)/step;
}


BOOST_AUTO_TEST_CASE(CoMJacobianDummyTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	typedef Eigen::Matrix<double, 1, 1> EScalar;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	Body b0(RBInertiad(EScalar::Random()(0)*10., h, I), 0, "b0");
	Body b1(RBInertiad(EScalar::Random()(0)*10., h, I), 1, "b1");
	Body b2(RBInertiad(EScalar::Random()(0)*10., h, I), 2, "b2");
	Body b3(RBInertiad(EScalar::Random()(0)*10., h, I), 3, "b3");
	Body b4(RBInertiad(EScalar::Random()(0)*10., h, I), 4, "b4");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);
	mbg.addBody(b4);

	Joint j0(Joint::RevX, true, 0, "j0");
	Joint j1(Joint::RevY, true, 1, "j1");
	Joint j2(Joint::RevZ, true, 2, "j2");
	Joint j3(Joint::Spherical, true, 3, "j3");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);

	//                b4
	//             j3 | Spherical
	//  Root     j0   |   j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ


	PTransformd to(Vector3d(0., 0.5, 0.));
	PTransformd from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);
	mbg.linkBodies(1, PTransformd(Vector3d(0.5, 0., 0.)),
								 4, PTransformd(Vector3d(-0.5, 0., 0.)), 3);

	MultiBody mb = mbg.makeMultiBody(0, true);
	CoMJacobianDummy comJac(mb);

	MultiBodyConfig mbc(mb);


	/**
		*						Test jacobian with the com speed get by differentiation.
		*						Also test computeCoMVelocity and computeCoMAcceleration.
		*/


	mbc.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	forwardKinematics(mb, mbc);

	auto testJacCoMVelAcc = [](const rbd::MultiBody& mb,
			rbd::MultiBodyConfig& mbc, rbd::CoMJacobianDummy& comJac)
	{
		forwardVelocity(mb, mbc);
		forwardAcceleration(mb, mbc);

		Vector3d CoMVel = computeCoMVelocity(mb, mbc);
		Vector3d CoMAcc = computeCoMAcceleration(mb, mbc);
		Vector3d CDot_diff = makeCoMDotFromStep(mb, mbc);
		MatrixXd CJac = comJac.jacobian(mb, mbc);
		MatrixXd CJacDot = comJac.jacobianDot(mb, mbc);

		BOOST_CHECK_EQUAL(CJac.rows(), 6);
		BOOST_CHECK_EQUAL(CJac.cols(), mb.nrDof());

		VectorXd alpha = dofToVector(mb, mbc.alpha);
		VectorXd alphaD = dofToVector(mb, mbc.alphaD);

		Vector3d CDot = CJac.block(3, 0, 3, mb.nrDof())*alpha;
		Vector3d CDotDot = CJac.block(3, 0, 3, mb.nrDof())*alphaD +
				CJacDot.block(3, 0, 3, mb.nrDof())*alpha;

		BOOST_CHECK_SMALL((CDot_diff - CDot).norm(), TOL);
		BOOST_CHECK_SMALL((CDot_diff - CoMVel).norm(), TOL);
		BOOST_CHECK_SMALL((CDotDot - CoMAcc).norm(), TOL);
	};

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			mbc.alphaD[i][j] = 1.;

			testJacCoMVelAcc(mb, mbc, comJac);

			mbc.alpha[i][j] = 0.;
			mbc.alphaD[i][j] = 0.;
		}
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			mbc.alphaD[i][j] = 1.;

			testJacCoMVelAcc(mb, mbc, comJac);
		}
	}
	mbc.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};


	/**
		* Same test but with a different q.
		*/


	Quaterniond q;
	q = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{}, {0.4}, {0.2}, {-0.1}, {q.w(), q.x(), q.y(), q.z()}};
	forwardKinematics(mb, mbc);

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			mbc.alphaD[i][j] = 1.;

			testJacCoMVelAcc(mb, mbc, comJac);

			mbc.alpha[i][j] = 0.;
			mbc.alphaD[i][j] = 0.;
		}
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			mbc.alphaD[i][j] = 1.;

			testJacCoMVelAcc(mb, mbc, comJac);
		}
	}
	mbc.alphaD = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	// test safe functions

	mbc.bodyPosW = {I, I, I};
	BOOST_CHECK_THROW(comJac.sJacobian(mb, mbc), std::domain_error);
	mbc = MultiBodyConfig(mb);


	/**
		*						Test jacobianDot with the jacobianDot get by differentiation.
		*/

	mbc.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	forwardKinematics(mb, mbc);

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);
			forwardAcceleration(mb, mbc);

			MatrixXd jacDot_diff = makeJDotFromStep(mb, mbc, comJac);
			MatrixXd jacDot = comJac.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDot.rows(), 6);
			BOOST_CHECK_EQUAL(jacDot.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDot_diff - jacDot).norm(), TOL);
			mbc.alpha[i][j] = 0.;
		}
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);
			forwardAcceleration(mb, mbc);

			MatrixXd jacDot_diff = makeJDotFromStep(mb, mbc, comJac);
			MatrixXd jacDot = comJac.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDot.rows(), 6);
			BOOST_CHECK_EQUAL(jacDot.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDot_diff - jacDot).norm(), TOL);
			mbc.alpha[i][j] = 0.;
		}
	}
	mbc.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};


	/**
		* Same test but with a different q.
		*/


	q = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{}, {0.4}, {0.2}, {-0.1}, {q.w(), q.x(), q.y(), q.z()}};
	forwardKinematics(mb, mbc);


	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);
			forwardAcceleration(mb, mbc);

			MatrixXd jacDot_diff = makeJDotFromStep(mb, mbc, comJac);
			MatrixXd jacDot = comJac.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDot.rows(), 6);
			BOOST_CHECK_EQUAL(jacDot.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDot_diff - jacDot).norm(), TOL);
			mbc.alpha[i][j] = 0.;
		}
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);
			forwardAcceleration(mb, mbc);

			MatrixXd jacDot_diff = makeJDotFromStep(mb, mbc, comJac);
			MatrixXd jacDot = comJac.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDot.rows(), 6);
			BOOST_CHECK_EQUAL(jacDot.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDot_diff - jacDot).norm(), TOL);
			mbc.alpha[i][j] = 0.;
		}
	}
	mbc.alpha = {{}, {0.}, {0.}, {0.}, {0., 0., 0.}};

	// test safe functions

	mbc.bodyPosW = {I, I, I};
	BOOST_CHECK_THROW(comJac.sJacobianDot(mb, mbc), std::domain_error);
	mbc = MultiBodyConfig(mb);

	MotionVecd mv;
	mbc.bodyVelB = {mv, mv, mv};
	BOOST_CHECK_THROW(comJac.sJacobianDot(mb, mbc), std::domain_error);
	mbc = MultiBodyConfig(mb);

	mbc.bodyVelW = {mv, mv, mv};
	BOOST_CHECK_THROW(comJac.sJacobianDot(mb, mbc), std::domain_error);
	mbc = MultiBodyConfig(mb);
}
