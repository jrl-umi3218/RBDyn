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
#define BOOST_TEST_MODULE Jacobian
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/FA.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"
#include "RBDyn/EulerIntegration.h"

// Arm
#include "XYZSarm.h"
#include "SSSarm.h"

const double TOL = 0.0000001;

void checkMultiBodyEq(const rbd::MultiBody& mb, std::vector<rbd::Body> bodies,
	std::vector<rbd::Joint> joints, std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent, std::vector<sva::PTransformd> Xt)
{
	// bodies
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.bodies().begin(), mb.bodies().end(),
																bodies.begin(), bodies.end());
	// joints
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.joints().begin(), mb.joints().end(),
																joints.begin(), joints.end());
	// pred
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.predecessors().begin(), mb.predecessors().end(),
																pred.begin(), pred.end());
	// succ
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.successors().begin(), mb.successors().end(),
																succ.begin(), succ.end());
	// parent
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.parents().begin(), mb.parents().end(),
																parent.begin(), parent.end());

	// Xt
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.transforms().begin(),
		mb.transforms().end(),
		Xt.begin(), Xt.end());

	// nrBodies
	BOOST_CHECK_EQUAL(mb.nrBodies(), bodies.size());
	// nrJoints
	BOOST_CHECK_EQUAL(mb.nrJoints(), bodies.size());

	int params = 0, dof = 0;
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		params += joints[i].params();
		dof += joints[i].dof();
	}

	BOOST_CHECK_EQUAL(params, mb.nrParams());
	BOOST_CHECK_EQUAL(dof, mb.nrDof());
}



BOOST_AUTO_TEST_CASE(JacobianConstructTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm();

	Jacobian jac1(mb, "b3");
	Jacobian jac2(mb, "b4");

	// test jointsPath
	std::vector<int> jointPath1 = {0, 1, 2, 3};
	std::vector<int> jointPath2 = {0, 1, 4};

	BOOST_CHECK_EQUAL_COLLECTIONS(jointPath1.begin(), jointPath1.end(),
		jac1.jointsPath().begin(), jac1.jointsPath().end());
	BOOST_CHECK_EQUAL_COLLECTIONS(jointPath2.begin(), jointPath2.end(),
		jac2.jointsPath().begin(), jac2.jointsPath().end());

	// test subMultibody
	MultiBody chain1 = jac1.subMultiBody(mb);
	MultiBody chain2 = jac2.subMultiBody(mb);

	// chain 1
	std::vector<Body> bodies = {mb.body(0), mb.body(1), mb.body(2), mb.body(3)};

	std::vector<Joint> joints = {Joint(Joint::Fixed, true, "Root"),
					mb.joint(1), mb.joint(2), mb.joint(3)};

	std::vector<int> pred = {-1, 0, 1, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 1, 2};

	PTransformd I(PTransformd::Identity());
	PTransformd to(Vector3d(0., 0.5, 0.));
	PTransformd unitY(Vector3d(0., 1., 0.));
	std::vector<PTransformd> Xt = {I, to, unitY, unitY};


	checkMultiBodyEq(chain1, bodies, joints, pred, succ, parent, Xt);

	// chain 2
	bodies = {mb.body(0), mb.body(1), mb.body(4)};

	joints = {Joint(Joint::Fixed, true, "Root"), mb.joint(1), mb.joint(4)};

	pred = {-1, 0, 1};
	succ = {0, 1, 2};
	parent = {-1, 0, 1};

	Xt = {I, to, PTransformd(Vector3d(0.5, 0.5, 0.))};


	checkMultiBodyEq(chain2, bodies, joints, pred, succ, parent, Xt);


	// test subMultiBody safe version
	BOOST_CHECK_THROW(jac1.sSubMultiBody(chain2), std::domain_error);
}


void checkJacobianMatrixFromVelocity(const rbd::MultiBody& subMb,
	rbd::MultiBodyConfig& subMbc,
	const std::vector<sva::MotionVecd>& velVec,
	const Eigen::MatrixXd& jacMat)
{
	int col = 0;
	for(int i = 0; i < subMb.nrJoints(); ++i)
	{
		for(int j = 0; j < subMb.joint(i).dof(); ++j)
		{
			subMbc.alpha[i][j] = 1.;

			forwardVelocity(subMb, subMbc);

			Eigen::Vector6d mv = velVec.back().vector();
			BOOST_CHECK_SMALL((mv - jacMat.col(col)).norm(), TOL);

			subMbc.alpha[i][j] = 0.;
			++col;
		}
	}
}


void checkJacobianMatrixSize(const rbd::MultiBody& subMb,
	const Eigen::MatrixXd& jacMat)
{
	// test jacobian size
	BOOST_CHECK_EQUAL(jacMat.rows(), 6);
	BOOST_CHECK_EQUAL(jacMat.cols(), subMb.nrDof());
}


void checkFullJacobianMatrix(const rbd::MultiBody& mb,
	const rbd::MultiBody& subMb, const rbd::Jacobian& jac,
	const Eigen::MatrixXd& jacMat)
{
	using namespace Eigen;

	MatrixXd fakeFull1(5, mb.nrDof());
	MatrixXd fakeFull2(6, mb.nrDof() + 1);
	MatrixXd fullJacMat(6, mb.nrDof());
	BOOST_CHECK_THROW(jac.sFullJacobian(mb, jacMat, fakeFull1), std::domain_error);
	BOOST_CHECK_THROW(jac.sFullJacobian(mb, jacMat, fakeFull2), std::domain_error);
	BOOST_CHECK_NO_THROW(jac.sFullJacobian(mb, jacMat, fullJacMat));

	for(int i = 0; i < subMb.nrJoints(); ++i)
	{
		int joint = jac.jointsPath()[i];
		int dof = mb.joint(i).dof();
		BOOST_CHECK_EQUAL(jacMat.block(0, subMb.jointPosInDof(i), 6, dof),
			fullJacMat.block(0, mb.jointPosInDof(joint), 6, dof));
	}
}


void checkJacobian(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc,
	rbd::Jacobian& jac)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	const MatrixXd& jac_mat = jac.jacobian(mb, mbc);
	MultiBody subMb = jac.subMultiBody(mb);

	// fill subMbc
	MultiBodyConfig subMbc(subMb);
	for(int i = 0; i < subMb.nrJoints(); ++i)
	{
		subMbc.bodyPosW[i] = mbc.bodyPosW[jac.jointsPath()[i]];
		subMbc.jointConfig[i] = mbc.jointConfig[jac.jointsPath()[i]];
		subMbc.parentToSon[i] = mbc.parentToSon[jac.jointsPath()[i]];
	}

	// test fullJacobian
	checkFullJacobianMatrix(mb, subMb, jac, jac_mat);

	// test jacobian
	const MatrixXd& jac_mat_w = jac.jacobian(mb, mbc);
	checkJacobianMatrixSize(subMb, jac_mat_w);
	checkJacobianMatrixFromVelocity(subMb, subMbc, subMbc.bodyVelW, jac_mat_w);

	// test bodyJacobian
	const MatrixXd& jac_mat_b = jac.bodyJacobian(mb, mbc);
	checkJacobianMatrixSize(subMb, jac_mat_b);
	checkJacobianMatrixFromVelocity(subMb, subMbc, subMbc.bodyVelB, jac_mat_w);
}



BOOST_AUTO_TEST_CASE(JacobianComputeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm();

	Jacobian jac1(mb, "b3");
	Jacobian jac2(mb, "b4");

	mbc.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);
	checkJacobian(mb, mbc, jac2);

	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);
	checkJacobian(mb, mbc, jac2);


	// test jacobian safe version
	MultiBodyConfig mbcBadNrBodyPos(mbc);
	mbcBadNrBodyPos.bodyPosW.resize(1);
	BOOST_CHECK_THROW(jac1.sJacobian(mb, mbcBadNrBodyPos), std::domain_error);

	MultiBodyConfig mbcBadNrJointConf(mbc);
	mbcBadNrJointConf.motionSubspace.resize(1);
	BOOST_CHECK_THROW(jac1.sJacobian(mb, mbcBadNrJointConf), std::domain_error);

	MultiBody mbErr = jac2.subMultiBody(mb);
	MultiBodyConfig mbcErr(mbErr);
	BOOST_CHECK_THROW(jac1.sJacobian(mbErr, mbcErr), std::domain_error);
}



BOOST_AUTO_TEST_CASE(JacobianComputeTestFreeFlyer)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm(false);

	Jacobian jac1(mb, "b3");
	Jacobian jac2(mb, "b4");

	Quaterniond quat(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*
		AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ()));
	Vector3d tran = Vector3d::Random()*10.;

	mbc.q = {{quat.w(), quat.x(), quat.y(), quat.z(), tran.x(), tran.y(), tran.z()},
					 {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);
	checkJacobian(mb, mbc, jac2);

	quat = (AngleAxisd(cst::pi<double>()/8., Vector3d::UnitX())*
		AngleAxisd(cst::pi<double>()/2., Vector3d::UnitZ()));
	tran = Vector3d::Random()*10.;
	mbc.q = {{quat.w(), quat.x(), quat.y(), quat.z(), tran.x(), tran.y(), tran.z()},
					 {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);
	checkJacobian(mb, mbc, jac2);


	// test jacobian safe version
	MultiBodyConfig mbcBadNrBodyPos(mbc);
	mbcBadNrBodyPos.bodyPosW.resize(1);
	BOOST_CHECK_THROW(jac1.sJacobian(mb, mbcBadNrBodyPos), std::domain_error);

	MultiBodyConfig mbcBadNrJointConf(mbc);
	mbcBadNrJointConf.motionSubspace.resize(1);
	BOOST_CHECK_THROW(jac1.sJacobian(mb, mbcBadNrJointConf), std::domain_error);

	MultiBody mbErr = jac2.subMultiBody(mb);
	MultiBodyConfig mbcErr(mbErr);
	BOOST_CHECK_THROW(jac1.sJacobian(mbErr, mbcErr), std::domain_error);
}



BOOST_AUTO_TEST_CASE(JacobianComputeTest2)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeSSSarm(true);

	Jacobian jac1(mb, "b3");

	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);


	Quaterniond q1(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX()));
	Quaterniond q2 = Quaterniond::Identity();
	Quaterniond q3 = Quaterniond::Identity();
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);


	q1 = AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*AngleAxisd(cst::pi<double>()/4., Vector3d::UnitY());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);


	q2 = AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);


	q3 = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobian(mb, mbc, jac1);
}


typedef std::function<const Eigen::MatrixXd&(const rbd::MultiBody&,
																							const rbd::MultiBodyConfig&)>
				jacobianComputeFunc;


Eigen::MatrixXd makeJDotFromStep(const rbd::MultiBody& mb,
													const rbd::MultiBodyConfig& mbc,
													jacobianComputeFunc jacComp)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double step = 1e-8;

	MultiBodyConfig mbcTmp(mbc);

	MatrixXd oJ = jacComp(mb, mbcTmp);
	eulerIntegration(mb, mbcTmp, step);
	forwardKinematics(mb, mbcTmp);
	forwardVelocity(mb, mbcTmp);
	MatrixXd nJ = jacComp(mb, mbcTmp);

	return (nJ - oJ)/step;
}

typedef const Eigen::MatrixXd& (rbd::Jacobian::*worldJacobian_t)
	(const rbd::MultiBody&, const rbd::MultiBodyConfig&);
typedef sva::MotionVecd (rbd::Jacobian::*worldVelocity_t)
	(const rbd::MultiBody&, const rbd::MultiBodyConfig&) const;

void testJacobianDot(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc,
	rbd::Jacobian& jac)
{
	using namespace std::placeholders; // bind
	using namespace Eigen;

	MatrixXd JD_diff = makeJDotFromStep(mb, mbc,
		std::bind(worldJacobian_t(&rbd::Jacobian::jacobian), jac, _1, _2));
	MatrixXd JD = jac.jacobianDot(mb, mbc);

	BOOST_CHECK_SMALL((JD_diff - JD).norm(), 2e-5);

	MatrixXd JD_diff_b = makeJDotFromStep(mb, mbc,
		std::bind(&rbd::Jacobian::bodyJacobian, jac, _1, _2));
	MatrixXd JD_b = jac.bodyJacobianDot(mb, mbc);

	BOOST_CHECK_SMALL((JD_diff_b - JD_b).norm(), 2e-5);
}


BOOST_AUTO_TEST_CASE(JacobianDotComputeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeSSSarm(true);

	Jacobian jac1(mb, "b3");

	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mb, mbc);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jac1);

			mbc.alpha[i][j] = 0.;
		}
	}


	Quaterniond q1 = AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*AngleAxisd(cst::pi<double>()/4., Vector3d::UnitY());
	Quaterniond q2 = Quaterniond::Identity();
	Quaterniond q3 = Quaterniond::Identity();
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jac1);

			mbc.alpha[i][j] = 0.;
		}
	}


	q2 = AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jac1);

			mbc.alpha[i][j] = 0.;
		}
	}


	q3 = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jac1);

			mbc.alpha[i][j] = 0.;
		}
	}


	// test with all joint velocity
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jac1);
		}
	}
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};


	// test with a point
	Jacobian jacP(mb, "b3", Vector3d::Random()*10.);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			testJacobianDot(mb, mbc, jacP);
		}
	}
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};


	// test with free flyier

	MultiBody mbF = mbg.makeMultiBody("b0", false);

	MultiBodyConfig mbcF(mbF);

	Jacobian jacF(mbF, "b3");

	mbcF.q = {{1., 0., 0., 0., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbcF.alphaD = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mbF, mbcF);

	for(int i = 0; i < mbF.nrJoints(); ++i)
	{
		for(int j = 0; j < mbF.joint(i).dof(); ++j)
		{
			mbcF.alpha[i][j] = 1.;
			forwardVelocity(mbF, mbcF);

			testJacobianDot(mbF, mbcF, jacF);
		}
	}
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};


	Quaterniond qF = AngleAxisd(1.2, Vector3d::UnitX())*AngleAxisd(-0.4, Vector3d::UnitZ());
	mbc.q = {{qF.w(), qF.x(), qF.y(), qF.z(), 1., 2., 3.},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mbF, mbcF);

	for(int i = 0; i < mbF.nrJoints(); ++i)
	{
		for(int j = 0; j < mbF.joint(i).dof(); ++j)
		{
			mbcF.alpha[i][j] = 1.;
			forwardVelocity(mbF, mbcF);

			testJacobianDot(mbF, mbcF, jacF);
		}
	}
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
}


void testTranslateJacobian(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc, const Eigen::Vector3d& P,
	rbd::Jacobian& jacO, rbd::Jacobian& jacP)
{
	using namespace Eigen;

	MatrixXd JO_w = jacO.jacobian(mb, mbc);
	MatrixXd JP_w = jacP.jacobian(mb, mbc);

	MatrixXd JO_P_w = JO_w;
	jacO.translateJacobian(JO_w, mbc, P, JO_P_w);

	BOOST_CHECK_SMALL((JO_P_w - JP_w).norm(), TOL);

	MatrixXd JO_b = jacO.jacobian(mb, mbc);
	MatrixXd JP_b = jacP.jacobian(mb, mbc);

	MatrixXd JO_P_b = JO_b;
	jacO.translateJacobian(JO_b, mbc, P, JO_P_b);

	BOOST_CHECK_SMALL((JO_P_b - JP_b).norm(), TOL);
}


BOOST_AUTO_TEST_CASE(JacobianTranslateTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeSSSarm(true);

	Vector3d point = Vector3d::Random()*10.;

	Jacobian jacO(mb, "b3");
	Jacobian jacP(mb, "b3", point);


	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	testTranslateJacobian(mb, mbc, point, jacO, jacP);


	Quaterniond q1 = AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*AngleAxisd(cst::pi<double>()/4., Vector3d::UnitY());
	Quaterniond q2(AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX()));
	Quaterniond q3(AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ()));
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	testTranslateJacobian(mb, mbc, point, jacO, jacP);
}


Eigen::MatrixXd jacobianVecBodyFromJac(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc, rbd::Jacobian& jac, const Eigen::Vector3d& vec)
{
	Eigen::MatrixXd jacO = jac.bodyJacobian(mb, mbc);
	Eigen::MatrixXd jacV(jacO.rows(), jacO.cols());
	jac.translateBodyJacobian(jacO, mbc, vec, jacV);

	return jacV - jacO;
}


Eigen::MatrixXd jacobianVecFromJac(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc, rbd::Jacobian& jac, const Eigen::Vector3d& vec)
{
	Eigen::MatrixXd jacO = jac.jacobian(mb, mbc);
	Eigen::MatrixXd jacV(jacO.rows(), jacO.cols());
	jac.translateJacobian(jacO, mbc, vec, jacV);

	return jacV - jacO;
}


BOOST_AUTO_TEST_CASE(JacobianVectorTest)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm(false);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	rbd::Jacobian jac1(mb, "b3");
	rbd::Jacobian jac2(mb, "b4", Eigen::Vector3d(0.1, -0.1, 0.2));

	for(int i = 0; i < 50; ++i)
	{
		Eigen::VectorXd q(mb.nrParams());
		q.setRandom();
		// normalize free flyier and spherical joint
		q.head<4>().normalize();
		q.segment(mb.jointPosInParam(mb.jointIndexByName("j3")), 4).normalize();
		rbd::vectorToParam(q, mbc.q);
		forwardKinematics(mb, mbc);

		Eigen::Vector3d vec;
		vec.setRandom();

		// vectorBody
		Eigen::MatrixXd jac1MatDiff = jacobianVecBodyFromJac(mb, mbc, jac1, vec);
		Eigen::MatrixXd jac1Mat = jac1.vectorBodyJacobian(mb, mbc, vec);


		BOOST_CHECK_SMALL((jac1MatDiff.block(3, 0, 3, jac1.dof()) -\
											 jac1Mat.block(3, 0, 3, jac1.dof())).norm(), TOL);

		Eigen::MatrixXd jac2MatDiff = jacobianVecBodyFromJac(mb, mbc, jac2, vec);
		Eigen::MatrixXd jac2Mat = jac2.vectorBodyJacobian(mb, mbc, vec);


		BOOST_CHECK_SMALL((jac2MatDiff.block(3, 0, 3, jac2.dof()) -\
											 jac2Mat.block(3, 0, 3, jac2.dof())).norm(), TOL);

		// vector
		Eigen::MatrixXd jac3MatDiff = jacobianVecFromJac(mb, mbc, jac1, vec);
		Eigen::MatrixXd jac3Mat = jac1.vectorJacobian(mb, mbc, vec);


		BOOST_CHECK_SMALL((jac3MatDiff.block(3, 0, 3, jac1.dof()) -\
											 jac3Mat.block(3, 0, 3, jac1.dof())).norm(), TOL);

		Eigen::MatrixXd jac4MatDiff = jacobianVecFromJac(mb, mbc, jac2, vec);
		Eigen::MatrixXd jac4Mat = jac2.vectorJacobian(mb, mbc, vec);


		BOOST_CHECK_SMALL((jac4MatDiff.block(3, 0, 3, jac2.dof()) -\
											 jac4Mat.block(3, 0, 3, jac2.dof())).norm(), TOL);
	}
}


template <typename MatFunc, typename VectorFunc>
Eigen::Vector6d testMatrixAgainstVector(
	const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc,
	const rbd::Jacobian& jac,
	MatFunc&& matFunc, const Eigen::VectorXd& alpha,
	VectorFunc&& vectorFunc)
{
	const Eigen::MatrixXd& mat = std::forward<MatFunc>(matFunc)(mb, mbc);
	Eigen::MatrixXd matFull(6, mb.nrDof());
	jac.fullJacobian(mb, mat, matFull);
	Eigen::Vector6d res = matFull*alpha;
	sva::MotionVecd mv = std::forward<VectorFunc>(vectorFunc)(mb, mbc);

	return mv.vector() - res;
}


BOOST_AUTO_TEST_CASE(JacobianVectorVelAccComputeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	using namespace std::placeholders;
	namespace cst = boost::math::constants;

	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm(false);

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	Jacobian jac1(mb, "b3");
	Jacobian jac2(mb, "b4");

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

		Vector3d point(Vector3d::Random());
		for(Jacobian& jac: {std::ref(jac1), std::ref(jac2)})
		{
			jac.point(point);

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(worldJacobian_t(&Jacobian::jacobian), std::ref(jac),
																					_1, _2), alpha,
																std::bind(worldVelocity_t(&Jacobian::velocity), std::ref(jac),
																					_1, _2)).norm(),
				TOL);

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(&Jacobian::bodyJacobian, std::ref(jac),
																					_1, _2), alpha,
																std::bind(&Jacobian::bodyVelocity, std::ref(jac),
																					_1, _2)).norm(),
				TOL);

			typedef MotionVecd (Jacobian::*normalAccel_func1)
				(const MultiBody&, const MultiBodyConfig&) const;
			typedef MotionVecd (Jacobian::*normalAccel_func2)
				(const MultiBody&, const MultiBodyConfig&, const std::vector<sva::MotionVecd>&) const;

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(&Jacobian::jacobianDot, std::ref(jac),
																					_1, _2), alpha,
																std::bind(static_cast<normalAccel_func1>
																					(&Jacobian::normalAcceleration),
																					std::ref(jac), _1, _2)).norm(),
				TOL);

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(&Jacobian::bodyJacobianDot, std::ref(jac),
																					_1, _2), alpha,
																std::bind(static_cast<normalAccel_func1>
																					(&Jacobian::bodyNormalAcceleration),
																					std::ref(jac), _1, _2)).norm(),
				TOL);

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(&Jacobian::jacobianDot, std::ref(jac),
																					_1, _2), alpha,
																std::bind(static_cast<normalAccel_func2>
																					(&Jacobian::normalAcceleration),
																					std::ref(jac), _1, _2,
																					std::ref(mbc.bodyAccB))).norm(),
				TOL);

			BOOST_CHECK_SMALL(
				testMatrixAgainstVector(mb, mbc, jac,
																std::bind(&Jacobian::bodyJacobianDot, std::ref(jac),
																					_1, _2), alpha,
																std::bind(static_cast<normalAccel_func2>
																					(&Jacobian::bodyNormalAcceleration),
																					std::ref(jac), _1, _2,
																					std::ref(mbc.bodyAccB))).norm(),
				TOL);
		}
	}
}
