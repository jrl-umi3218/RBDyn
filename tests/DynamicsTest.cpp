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
#define BOOST_TEST_MODULE Dynamics
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "FK.h"
#include "FV.h"
#include "FD.h"
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
								 1, PTransform::Identity(), 0);

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

	double torque = Vector3d(0., 0., 0.5).cross(Vector3d(0., 9.81, 0.))(0);
	BOOST_CHECK_SMALL(std::abs(torque - mbc.jointTorque[1][0]), TOL);

	MultiBodyConfig mbc2 = mbc;
	mbc2.gravity = Vector3d::Zero();
	MotionVec gravity(Vector3d::Zero(), Vector3d(0., 9.81, 0.));
	ForceVec gravityB1 = mb.body(0).inertia()*(mbc2.bodyPosW[0]*gravity);
	ForceVec gravityB2 = mb.body(1).inertia()*(mbc2.bodyPosW[1]*gravity);
	mbc2.force = {gravityB1,
								gravityB2};
	id.inverseDynamics(mb, mbc2);

	torque = Vector3d(0., 0., 0.5).cross(Vector3d(0., -9.81, 0.))(0);
	BOOST_CHECK_SMALL(std::abs(torque - mbc2.jointTorque[1][0]), TOL);
}

void makeRandomVecVec(std::vector<std::vector<double>>& vec)
{
	typedef Eigen::Matrix<double, 1, 1> EScalar;
	for(auto& v1: vec)
		for(auto& v2: v1)
			v2 = EScalar::Random()(0)*10.;
}

void normalizeQuat(std::vector<double>& q)
{
	Eigen::Vector4d qv(q[0], q[1], q[2], q[3]);
	double norm = qv.norm();
	for(int i = 0; i < 4; ++i)
		q[i] /= norm;
}

void makeRandomConfig(rbd::MultiBodyConfig& mbc)
{
	makeRandomVecVec(mbc.q);
	makeRandomVecVec(mbc.alpha);
	makeRandomVecVec(mbc.alphaD);
	makeRandomVecVec(mbc.jointTorque);

	for(std::size_t i = 0; i < mbc.q.size(); ++i)
	{
		if(mbc.q[i].size() == 4 || mbc.q[i].size() == 7)
		{
			normalizeQuat(mbc.q[i]);
		}
	}
}

Eigen::MatrixXd makeHFromID(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc,
	rbd::InverseDynamics& id,
	const Eigen::VectorXd& C)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	Eigen::MatrixXd H(mb.nrDof(), mb.nrDof());
	VectorXd Hd(mb.nrDof());

	MultiBodyConfig mbcd(mbc);
	for(auto& v1: mbcd.alphaD)
	{
		for(auto& v2: v1)
		{
			v2 = 0.;
		}
	}

	int col = 0;
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		for(int j = 0; j < mb.joint(i).dof(); ++j)
		{
			mbcd.alphaD[i][j] = 1.;

			id.inverseDynamics(mb, mbcd);

			int dof = 0;
			for(auto& v1: mbcd.jointTorque)
			{
				for(auto& v2: v1)
				{
					Hd(dof) = v2;
					++dof;
				}
			}

			H.col(col) = Hd - C;

			mbcd.alphaD[i][j] = 0.;
			++col;
		}
	}

	return H;
}



BOOST_AUTO_TEST_CASE(IDvsFDFixed)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	typedef Matrix<double, 1, 1> EScalar;

	RBInertia I0(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I1(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I2(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I3(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());

	Body b0(I0, 0, "b0");
	Body b1(I1, 1, "b1");
	Body b2(I2, 2, "b2");
	Body b3(I3, 3, "b3");

	Joint j0 = Joint(Joint::Spherical, true, 0, "j0");
	Joint j1 = Joint(Joint::RevX, true, 1, "j1");
	Joint j2 = Joint(Joint::RevZ, true, 2, "j2");


	MultiBodyGraph mbg;

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies(0, PTransform(Vector3d(0., 0.5, 0.)),
								 1, PTransform(Vector3d(0., -0.5, 0.)), 0);
	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 2, PTransform(Vector3d(0., 0., 0.)), 1);
	mbg.linkBodies(1, PTransform(Vector3d(-0.5, 0., 0.)),
								 3, PTransform(Vector3d(0., 0., 0.)), 2);

	MultiBody mb = mbg.makeMultiBody(0, true);

	MultiBodyConfig mbc(mb);

	mbc.q = {{}, {1., 0., 0., 0.}, {0.}, {0.}};
	mbc.alpha = {{}, {0., 0., 0.}, {0.}, {0.}};
	mbc.alphaD = {{}, {0., 0., 0.}, {0.}, {0.}};
	mbc.force = {ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero())};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	InverseDynamics id(mb);
	ForwardDynamics fd(mb);


	// check non linear is null
	mbc.gravity = Vector3d::Zero();
	fd.computeC(mb, mbc);

	BOOST_CHECK(fd.C().isZero());


	// check FD C against ID C
	mbc.gravity = Vector3d(0., -9.81, 0.);
	makeRandomConfig(mbc);

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	fd.computeC(mb, mbc);

	mbc.alphaD = {{}, {0., 0., 0.}, {0.}, {0.}};
	id.inverseDynamics(mb, mbc);

	VectorXd ID_C(mb.nrDof());
	int dof = 0;
	for(auto& v1: mbc.jointTorque)
	{
		for(auto& v2: v1)
		{
			ID_C(dof) = v2;
			++dof;
		}
	}

	BOOST_CHECK_EQUAL(fd.C(), ID_C);


	// check FD H against ID H
	makeRandomConfig(mbc);

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	fd.forwardDynamics(mb, mbc);
	MatrixXd ID_H = makeHFromID(mb, mbc, id, fd.C());

	BOOST_CHECK_SMALL((fd.H() - ID_H).norm(), 1e-10);


	// check symmetry

	MatrixXd L = fd.H().triangularView<Lower>();
	MatrixXd U = fd.H().triangularView<Upper>().transpose();

	BOOST_CHECK_SMALL((L - U).norm(), 1e-10);


	// check torque and acceleration output

	// torque -> FD -> alphaD -> ID -> torque
	makeRandomConfig(mbc);

	VectorXd vT1(mb.nrDof()), vT2(mb.nrDof());

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	paramToVector(mbc.jointTorque, vT1);

	fd.forwardDynamics(mb, mbc);
	id.inverseDynamics(mb, mbc);

	paramToVector(mbc.jointTorque, vT2);

	BOOST_CHECK_SMALL((vT1 - vT2).norm(), 1e-10);


	// alphaD -> ID -> torque -> FD -> alphaD
	makeRandomConfig(mbc);

	VectorXd vA1(mb.nrDof()), vA2(mb.nrDof());

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	paramToVector(mbc.alphaD, vA1);

	id.inverseDynamics(mb, mbc);
	fd.forwardDynamics(mb, mbc);

	paramToVector(mbc.alphaD, vA2);

	BOOST_CHECK_SMALL((vA1 - vA2).norm(), 1e-10);
}




BOOST_AUTO_TEST_CASE(IDvsFDFree)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	typedef Matrix<double, 1, 1> EScalar;

	RBInertia I0(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I1(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I2(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());
	RBInertia I3(EScalar::Random()(0)*10., Vector3d::Random()*10.,
		Matrix3d::Random().triangularView<Lower>());

	Body b0(I0, 0, "b0");
	Body b1(I1, 1, "b1");
	Body b2(I2, 2, "b2");
	Body b3(I3, 3, "b3");

	Joint j0 = Joint(Joint::Spherical, true, 0, "j0");
	Joint j1 = Joint(Joint::RevX, true, 1, "j1");
	Joint j2 = Joint(Joint::RevZ, true, 2, "j2");


	MultiBodyGraph mbg;

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies(0, PTransform(Vector3d(0., 0.5, 0.)),
								 1, PTransform(Vector3d(0., -0.5, 0.)), 0);
	mbg.linkBodies(1, PTransform(Vector3d(0.5, 0., 0.)),
								 2, PTransform(Vector3d(0., 0., 0.)), 1);
	mbg.linkBodies(1, PTransform(Vector3d(-0.5, 0., 0.)),
								 3, PTransform(Vector3d(0., 0., 0.)), 2);

	MultiBody mb = mbg.makeMultiBody(0, false);

	MultiBodyConfig mbc(mb);

	mbc.q = {{1., 0., 0., 0., 0., 0., 0.}, {1., 0., 0., 0.}, {0.}, {0.}};
	mbc.alpha = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0.}, {0.}};
	mbc.alphaD = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0.}, {0.}};
	mbc.force = {ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero()),
							 ForceVec(Vector6d::Zero())};

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	InverseDynamics id(mb);
	ForwardDynamics fd(mb);


	// check non linear is null
	mbc.gravity = Vector3d::Zero();
	fd.computeC(mb, mbc);

	BOOST_CHECK(fd.C().isZero());


	// check FD C against ID C
	mbc.gravity = Vector3d(0., -9.81, 0.);
	makeRandomConfig(mbc);

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	fd.computeC(mb, mbc);

	mbc.alphaD = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0.}, {0.}, {0.}};
	id.inverseDynamics(mb, mbc);

	VectorXd ID_C(mb.nrDof());
	int dof = 0;
	for(auto& v1: mbc.jointTorque)
	{
		for(auto& v2: v1)
		{
			ID_C(dof) = v2;
			++dof;
		}
	}

	BOOST_CHECK_EQUAL(fd.C(), ID_C);


	// check FD H against ID H
	makeRandomConfig(mbc);

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	fd.forwardDynamics(mb, mbc);
	MatrixXd ID_H = makeHFromID(mb, mbc, id, fd.C());

	BOOST_CHECK_SMALL((fd.H() - ID_H).norm(), 1e-10);


	// check symmetry

	MatrixXd L = fd.H().triangularView<Lower>();
	MatrixXd U = fd.H().triangularView<Upper>().transpose();

	BOOST_CHECK_SMALL((L - U).norm(), 1e-10);


	// check torque and acceleration output

	// torque -> FD -> alphaD -> ID -> torque
	makeRandomConfig(mbc);

	VectorXd vT1(mb.nrDof()), vT2(mb.nrDof());

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	paramToVector(mbc.jointTorque, vT1);

	fd.forwardDynamics(mb, mbc);
	id.inverseDynamics(mb, mbc);

	paramToVector(mbc.jointTorque, vT2);

	BOOST_CHECK_SMALL((vT1 - vT2).norm(), 1e-10);


	// alphaD -> ID -> torque -> FD -> alphaD
	makeRandomConfig(mbc);

	VectorXd vA1(mb.nrDof()), vA2(mb.nrDof());

	forwardKinematics(mb, mbc);
	forwardVelocity(mb, mbc);

	paramToVector(mbc.alphaD, vA1);

	id.inverseDynamics(mb, mbc);
	fd.forwardDynamics(mb, mbc);

	paramToVector(mbc.alphaD, vA2);

	BOOST_CHECK_SMALL((vA1 - vA2).norm(), 1e-10);
}
