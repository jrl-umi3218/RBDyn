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
#include <SpaceVecAlg>

// RBDyn
#include "FK.h"
#include "FV.h"
#include "Jacobian.h"
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"
#include "EulerIntegration.h"

const double TOL = 0.0000001;

void checkMultiBodyEq(const rbd::MultiBody& mb, std::vector<rbd::Body> bodies,
	std::vector<rbd::Joint> joints, std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent, std::vector<sva::PTransform> Xt)
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

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertia rbi(mass, h, I);

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(rbi, 2, "b2");
	Body b3(rbi, 3, "b3");
	Body b4(rbi, 4, "b4");

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


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);
	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 4, PTransform(Vector3d(-0.5, 0., 0.)), 3);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	Jacobian jac1(mb, 3);
	Jacobian jac2(mb, 4);


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
	std::vector<Body> bodies = {b0, b1, b2, b3};

	std::vector<Joint> joints = {Joint(Joint::Fixed, true, -1, "Root"),
															 j0, j1, j2};

	std::vector<int> pred = {-1, 0, 1, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 1, 2};

	PTransform unitY(Vector3d(0., 1., 0.));
	std::vector<PTransform> Xt = {I, to, unitY, unitY};


	checkMultiBodyEq(chain1, bodies, joints, pred, succ, parent, Xt);

	// chain 2
	bodies = {b0, b1, b4};

	joints = {Joint(Joint::Fixed, true, -1, "Root"), j0, j3};

	pred = {-1, 0, 1};
	succ = {0, 1, 2};
	parent = {-1, 0, 1};

	Xt = {I, to, PTransform(Vector3d(0.5, 0.5, 0.))};


	checkMultiBodyEq(chain2, bodies, joints, pred, succ, parent, Xt);


	// test subMultiBody safe version
	BOOST_CHECK_THROW(jac1.sSubMultiBody(chain2), std::domain_error);
}



void checkJacobianMatrix(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc,
	rbd::Jacobian& jac)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	const MatrixXd& jac_mat = jac.jacobian(mb, mbc);
	MatrixXd full_jac_mat(6, mb.nrDof());
	MultiBody subMb = jac.subMultiBody(mb);

	// test jacobian size
	BOOST_CHECK_EQUAL(jac_mat.rows(), 6);
	BOOST_CHECK_EQUAL(jac_mat.cols(), subMb.nrDof());

	// test fullJacobian
	MatrixXd fake_full1(5, mb.nrDof());
	MatrixXd fake_full2(6, mb.nrDof() + 1);
	BOOST_CHECK_THROW(jac.sFullJacobian(mb, jac_mat, fake_full1), std::domain_error);
	BOOST_CHECK_THROW(jac.sFullJacobian(mb, jac_mat, fake_full2), std::domain_error);
	BOOST_CHECK_NO_THROW(jac.sFullJacobian(mb, jac_mat, full_jac_mat));

	MultiBodyConfig subMbc(subMb);
	for(std::size_t i = 0; i < subMb.nrJoints(); ++i)
	{
		subMbc.bodyPosW[i] = mbc.bodyPosW[jac.jointsPath()[i]];
		subMbc.jointConfig[i] = mbc.jointConfig[jac.jointsPath()[i]];
		subMbc.parentToSon[i] = mbc.parentToSon[jac.jointsPath()[i]];
	}

	int col = 0;
	for(std::size_t i = 0; i < subMb.nrJoints(); ++i)
	{
		for(int j = 0; j < subMb.joint(i).dof(); ++j)
		{
			subMbc.alpha[i][j] = 1.;

			forwardVelocity(subMb, subMbc);

			Vector6d mv = subMbc.bodyVelW.back().vector();
			BOOST_CHECK_SMALL((mv -jac_mat.col(col)).norm(), TOL);

			subMbc.alpha[i][j] = 0.;
			++col;
		}

		int joint = jac.jointsPath()[i];
		int dof = mb.joint(i).dof();
		BOOST_CHECK_EQUAL(jac_mat.block(0, subMb.jointPosInDof(i), 6, dof),
			full_jac_mat.block(0, mb.jointPosInDof(joint), 6, dof));
	}
}



BOOST_AUTO_TEST_CASE(JacobianComputeTest)
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
	Body b4(rbi, 4, "b4");

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


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);
	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 4, PTransform(Vector3d(-0.5, 0., 0.)), 3);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	Jacobian jac1(mb, 3);
	Jacobian jac2(mb, 4);

	mbc.q = {{}, {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);
	checkJacobianMatrix(mb, mbc, jac2);

	mbc.q = {{}, {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);
	checkJacobianMatrix(mb, mbc, jac2);


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

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertia rbi(mass, h, I);

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(rbi, 2, "b2");
	Body b3(rbi, 3, "b3");
	Body b4(rbi, 4, "b4");

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
	//  Free     RevX   RevY    RevZ


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);
	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 4, PTransform(Vector3d(-0.5, 0., 0.)), 3);

	MultiBody mb = mbg.makeMultiBody(0, false);

	MultiBodyConfig mbc(mb);

	Jacobian jac1(mb, 3);
	Jacobian jac2(mb, 4);

	Quaterniond quat(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*
		AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ()));
	Vector3d tran = Vector3d::Random()*10.;

	mbc.q = {{quat.w(), quat.x(), quat.y(), quat.z(), tran.x(), tran.y(), tran.z()},
					 {0.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);
	checkJacobianMatrix(mb, mbc, jac2);

	quat = (AngleAxisd(cst::pi<double>()/8., Vector3d::UnitX())*
		AngleAxisd(cst::pi<double>()/2., Vector3d::UnitZ()));
	tran = Vector3d::Random()*10.;
	mbc.q = {{quat.w(), quat.x(), quat.y(), quat.z(), tran.x(), tran.y(), tran.z()},
					 {cst::pi<double>()/2.}, {0.}, {0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);
	checkJacobianMatrix(mb, mbc, jac2);


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

	Joint j0(Joint::Spherical, true, 0, "j0");
	Joint j1(Joint::Spherical, true, 1, "j1");
	Joint j2(Joint::Spherical, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	//  Root     j0       j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    S       S       S


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, PTransform::Identity(), 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	Jacobian jac1(mb, 3);

	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);



	Quaterniond q1(AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX()));
	Quaterniond q2 = Quaterniond::Identity();
	Quaterniond q3 = Quaterniond::Identity();
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);



	q1 = AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*AngleAxisd(cst::pi<double>()/4., Vector3d::UnitY());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);



	q2 = AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);



	q3 = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	checkJacobianMatrix(mb, mbc, jac1);
}


Eigen::MatrixXd makeJDotFromStep(const rbd::MultiBody& mb,
													const rbd::MultiBodyConfig& mbc,
													rbd::Jacobian& jac)
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

BOOST_AUTO_TEST_CASE(JacobianDotComputeTest)
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

	Joint j0(Joint::Spherical, true, 0, "j0");
	Joint j1(Joint::Spherical, true, 1, "j1");
	Joint j2(Joint::Spherical, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	//  Root     j0       j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    S       S       S


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, PTransform::Identity(), 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	Jacobian jac1(mb, 3);

	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mb, mbc);
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jac1);
			MatrixXd JD = jac1.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), TOL);
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
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jac1);
			MatrixXd JD = jac1.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
			mbc.alpha[i][j] = 0.;
		}
	}



	q2 = AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jac1);
			MatrixXd JD = jac1.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
			mbc.alpha[i][j] = 0.;
		}
	}



	q3 = AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ());
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mb, mbc);
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jac1);
			MatrixXd JD = jac1.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
			mbc.alpha[i][j] = 0.;
		}
	}




	// test with all joint velocity
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jac1);
			MatrixXd JD = jac1.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
		}
	}
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};



	// test with a point
	Jacobian jacP(mb, 3, Vector3d::Random()*10.);
	for(std::size_t i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbc.alpha[i][j] = 1.;
			forwardVelocity(mb, mbc);

			MatrixXd JD_diff = makeJDotFromStep(mb, mbc, jacP);
			MatrixXd JD = jacP.jacobianDot(mb, mbc);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
		}
	}
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};



	// test with free flyier

	MultiBody mbF = mbg.makeMultiBody(0, false);

	MultiBodyConfig mbcF(mbF);

	Jacobian jacF(mbF, 3);

	mbcF.q = {{1., 0., 0., 0., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbcF.alphaD = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mbF, mbcF);

	for(std::size_t i = 0; i < mbF.nrJoints(); ++i)
	{
		for(int j = 0; j < mbF.joint(i).dof(); ++j)
		{
			mbcF.alpha[i][j] = 1.;
			forwardVelocity(mbF, mbcF);

			MatrixXd JD_diff = makeJDotFromStep(mbF, mbcF, jacF);
			MatrixXd JD = jacF.jacobianDot(mbF, mbcF);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
		}
	}
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};


	Quaterniond qF = AngleAxisd(1.2, Vector3d::UnitX())*AngleAxisd(-0.4, Vector3d::UnitZ());
	mbc.q = {{qF.w(), qF.x(), qF.y(), qF.z(), 1., 2., 3.},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};
	forwardKinematics(mbF, mbcF);

	for(std::size_t i = 0; i < mbF.nrJoints(); ++i)
	{
		for(int j = 0; j < mbF.joint(i).dof(); ++j)
		{
			mbcF.alpha[i][j] = 1.;
			forwardVelocity(mbF, mbcF);

			MatrixXd JD_diff = makeJDotFromStep(mbF, mbcF, jacF);
			MatrixXd JD = jacF.jacobianDot(mbF, mbcF);

			BOOST_CHECK_SMALL((JD_diff - JD).norm(), 1e-5);
		}
	}
	mbcF.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
}


BOOST_AUTO_TEST_CASE(JacobianTranslateTest)
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

	Joint j0(Joint::Spherical, true, 0, "j0");
	Joint j1(Joint::Spherical, true, 1, "j1");
	Joint j2(Joint::Spherical, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	//  Root     j0       j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Fixed    S       S       S


	PTransform to(Vector3d(0., 0.5, 0.));
	PTransform from(Vector3d(0., -0.5, 0.));


	mbg.linkBodies(0, PTransform::Identity(), 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	Vector3d point = Vector3d::Random()*10.;

	Jacobian jac0(mb, 3);
	Jacobian jacP(mb, 3, point);



	mbc.q = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {1., 0., 0., 0.}};
	mbc.alpha = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	mbc.alphaD = {{}, {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	MatrixXd J0 = jac0.jacobian(mb, mbc);
	MatrixXd JP = jacP.jacobian(mb, mbc);

	MatrixXd J0_P = J0;
	jac0.translateJacobian(J0, mbc, point, J0_P);

	BOOST_CHECK_SMALL((J0_P - JP).norm(), TOL);



	Quaterniond q1 = AngleAxisd(cst::pi<double>()/2., Vector3d::UnitX())*AngleAxisd(cst::pi<double>()/4., Vector3d::UnitY());
	Quaterniond q2(AngleAxisd(cst::pi<double>()/4., Vector3d::UnitX()));
	Quaterniond q3(AngleAxisd(cst::pi<double>()/8., Vector3d::UnitZ()));
	mbc.q = {{},
					 {q1.w(), q1.x(), q1.y(), q1.z()},
					 {q2.w(), q2.x(), q2.y(), q2.z()},
					 {q3.w(), q3.x(), q3.y(), q3.z()}};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	J0 = jac0.jacobian(mb, mbc);
	JP = jacP.jacobian(mb, mbc);

	jac0.translateJacobian(J0, mbc, point, J0_P);

	BOOST_CHECK_SMALL((J0_P - JP).norm(), TOL);
}
