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
#define BOOST_TEST_MODULE CoMTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/EulerIntegration.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/FA.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"
#include "RBDyn/CoM.h"

// arm
#include "XYZarm.h"

const double TOL = 0.000001;


std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph>
makeXYZSarmRandomCoM(bool isFixed=true)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	typedef Eigen::Matrix<double, 1, 1> EScalar;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Random();

	RBInertiad rbi(mass, h, I);

	Body b0(RBInertiad((fabs(EScalar::Random()(0))+1e-8)*10., h, I), "b0");
	Body b1(RBInertiad((fabs(EScalar::Random()(0))+1e-8)*10., h, I), "b1");
	Body b2(RBInertiad((fabs(EScalar::Random()(0))+1e-8)*10., h, I), "b2");
	Body b3(RBInertiad((fabs(EScalar::Random()(0))+1e-8)*10., h, I), "b3");
	Body b4(RBInertiad((fabs(EScalar::Random()(0))+1e-8)*10., h, I), "b4");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);
	mbg.addBody(b4);

	Joint j0(Joint::RevX, true, "j0");
	Joint j1(Joint::RevY, true, "j1");
	Joint j2(Joint::RevZ, true, "j2");
	Joint j3(Joint::Spherical, true, "j3");

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


	mbg.linkBodies("b0", to, "b1", from, "j0");
	mbg.linkBodies("b1", to, "b2", from, "j1");
	mbg.linkBodies("b2", to, "b3", from, "j2");
	mbg.linkBodies("b1", PTransformd(Vector3d(0.5, 0., 0.)),
			"b4", PTransformd(Vector3d(-0.5, 0., 0.)), "j3");

	MultiBody mb = mbg.makeMultiBody("b0", isFixed);

	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}



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

	Body b0(rbi, "b0");
	Body b1(rbi, "b1");
	Body b2(RBInertiad(2., h, I), "b2");
	Body b3(rbi, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::RevX, true, "j0");
	Joint j1(Joint::RevY, true, "j1");
	Joint j2(Joint::RevZ, true, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);


	//  Root     j0      j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    RevX   RevY    RevZ

	PTransformd to(Vector3d(0., 0.5, 0.));
	PTransformd from(Vector3d(0., 0., 0.));

	mbg.linkBodies("b0", to, "b1", from, "j0");
	mbg.linkBodies("b1", to, "b2", from, "j1");
	mbg.linkBodies("b2", to, "b3", from, "j2");


	MultiBody mb = mbg.makeMultiBody("b0", true);
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

	sva::PTransformd I(sva::PTransformd::Identity());

	MultiBodyGraph mbg;
	MultiBody mb;
	MultiBodyConfig mbc;

	std::tie(mb, mbc, mbg) = makeXYZSarmRandomCoM();

	CoMJacobianDummy comJac(mb);

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

		BOOST_CHECK_EQUAL(CJac.rows(), 3);
		BOOST_CHECK_EQUAL(CJac.cols(), mb.nrDof());

		VectorXd alpha = dofToVector(mb, mbc.alpha);
		VectorXd alphaD = dofToVector(mb, mbc.alphaD);

		Vector3d CDot = CJac*alpha;
		Vector3d CDotDot = CJac*alphaD + CJacDot*alpha;

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

			BOOST_CHECK_EQUAL(jacDot.rows(), 3);
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

			BOOST_CHECK_EQUAL(jacDot.rows(), 3);
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

			BOOST_CHECK_EQUAL(jacDot.rows(), 3);
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

			BOOST_CHECK_EQUAL(jacDot.rows(), 3);
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


BOOST_AUTO_TEST_CASE(CoMJacobianTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBodyGraph mbg;
	MultiBody mb;
	MultiBodyConfig mbc;

	std::tie(mb, mbc, mbg) = makeXYZSarmRandomCoM();

	std::vector<double> weight(mb.nrBodies());
	for(std::size_t i = 0; i < weight.size(); ++i)
	{
		weight[i] = Eigen::Matrix<double, 1, 1>::Random()(0);
	}

	CoMJacobian comJac(mb, weight);
	CoMJacobianDummy comJacDummy(mb, weight);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	// test jacobian
	MatrixXd jacMat = comJac.jacobian(mb, mbc);
	MatrixXd jacDummyMat = comJacDummy.jacobian(mb, mbc);

	BOOST_CHECK_EQUAL(jacMat.rows(), 3);
	BOOST_CHECK_EQUAL(jacMat.cols(), mb.nrDof());

	BOOST_CHECK_SMALL((jacMat - jacDummyMat).norm(), TOL);

	// change configuration
	Quaterniond q;
	q = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{}, {0.4}, {0.2}, {-0.1}, {q.w(), q.x(), q.y(), q.z()}};
	forwardKinematics(mb, mbc);

	jacMat = comJac.jacobian(mb, mbc);
	jacDummyMat = comJacDummy.jacobian(mb, mbc);

	BOOST_CHECK_SMALL((jacMat - jacDummyMat).norm(), TOL);


	// Test jacobianDot

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd jacDotMat = comJac.jacobianDot(mb, mbc);
			MatrixXd jacDotDummyMat = comJacDummy.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDotMat.rows(), 3);
			BOOST_CHECK_EQUAL(jacDotMat.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDotMat - jacDotDummyMat).norm(), TOL);
			mbc.alpha[i][j] = 0.;
		}
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd jacDotMat = comJac.jacobianDot(mb, mbc);
			MatrixXd jacDotDummyMat = comJacDummy.jacobianDot(mb, mbc);

			BOOST_CHECK_EQUAL(jacDotMat.rows(), 3);
			BOOST_CHECK_EQUAL(jacDotMat.cols(), mb.nrDof());

			BOOST_CHECK_SMALL((jacDotMat - jacDotDummyMat).norm(), TOL);
		}
	}


	// Test velocity and normal acceleration function
	for(int i = 0; i < 50; ++i)
	{
		Eigen::VectorXd q(mb.nrParams()), alpha(mb.nrDof());
		q.setRandom();
		alpha.setRandom();
		// normalize free flyier and spherical joint
		q.head<4>().normalize();
		q.segment(mb.jointPosInParam(mb.jointIndexByName("j3")), 4).normalize();
		rbd::vectorToParam(q, mbc.q);
		rbd::vectorToParam(alpha, mbc.alpha);
		forwardKinematics(mb, mbc);
		forwardVelocity(mb, mbc);
		// calcul the normal acceleration since alphaD is zero
		forwardAcceleration(mb, mbc);

		// test com velocity
		const MatrixXd& comJacMat = comJac.jacobian(mb, mbc);
		Vector3d velFromJac = comJacMat*alpha;
		Vector3d velFromMbc = comJac.velocity(mb, mbc);

		BOOST_CHECK_SMALL((velFromJac - velFromMbc).norm(), TOL);

		// test com normal acceleration
		const MatrixXd& comJacDotMat = comJac.jacobianDot(mb, mbc);
		Vector3d normalAccFromJac = comJacDotMat*alpha;
		Vector3d normalAccFromMbc1 = comJac.normalAcceleration(mb, mbc);
		Vector3d normalAccFromMbc2 = comJac.normalAcceleration(mb, mbc,
			mbc.bodyAccB);

		BOOST_CHECK_SMALL((normalAccFromJac - normalAccFromMbc1).norm(), TOL);
		BOOST_CHECK_SMALL((normalAccFromJac - normalAccFromMbc2).norm(), TOL);
	}

	// create a multibody with new inertial parameter to test updateInertialParameters
	std::tie(mb, mbc, mbg) = makeXYZSarmRandomCoM();

	MultiBodyGraph badMbg;
	MultiBody badMb;
	MultiBodyConfig badMbc;
	std::tie(badMb, badMbc, badMbg) = makeXYZarm();

	BOOST_CHECK_THROW(comJac.sUpdateInertialParameters(badMb), std::domain_error);
	BOOST_CHECK_NO_THROW(comJac.sUpdateInertialParameters(mb));
	CoMJacobianDummy comJacDummyUpdated(mb, weight);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	// test jacobian with updated model
	jacMat = comJac.jacobian(mb, mbc);
	jacDummyMat = comJacDummyUpdated.jacobian(mb, mbc);
	BOOST_CHECK_SMALL((jacMat - jacDummyMat).norm(), TOL);

	// test weight getter/setter
	std::vector<double> weight2 = comJac.weight();
	BOOST_CHECK_EQUAL_COLLECTIONS(weight.begin(), weight.end(),
		weight2.begin(), weight2.end());

	for(std::size_t i = 0; i < weight2.size(); ++i)
	{
		weight2[i] += 10.;
	}

	comJac.weight(mb, weight2);
	BOOST_CHECK_EQUAL_COLLECTIONS(weight2.begin(), weight2.end(),
		comJac.weight().begin(), comJac.weight().end());

	CoMJacobianDummy comJacDummyWeight2(mb, weight2);
	// test jacobian with new weight
	jacMat = comJac.jacobian(mb, mbc);
	jacDummyMat = comJacDummyWeight2.jacobian(mb, mbc);
	BOOST_CHECK_SMALL((jacMat - jacDummyMat).norm(), TOL);
}
