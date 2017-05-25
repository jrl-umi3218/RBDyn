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
#define BOOST_TEST_MODULE JointTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Joint.h"

const double TOL = 1e-10;

void testRevolute(rbd::Joint::OldType type, const Eigen::Vector3d& axis, bool forward)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, forward, "rev");
	double dir = forward ? 1. : -1;

	// test motion
	Vector6d S;
	S << dir*axis, 0., 0., 0.;

	// test motion
	MotionVecd motion(S);

	// test motion
	PTransformd rot90(AngleAxisd(-constants::pi<double>()/2., dir*axis).matrix());

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Rev);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.name(), "rev");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {0.};
	std::vector<double> zeroD = {0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	BOOST_CHECK_EQUAL(j.pose<double>({constants::pi<double>()/2.}), rot90);

	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

void testPrismatic(rbd::Joint::OldType type, const Eigen::Vector3d& axis, bool forward)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, forward, "prism");
	double dir = forward ? 1. : -1;

	// test motion
	Vector6d S;
	S << 0., 0., 0., dir*axis;

	// test motion
	MotionVecd motion(S);

	// test motion
	PTransformd trans2(Vector3d(dir*axis*2.));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Prism);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.name(), "prism");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {0.};
	std::vector<double> zeroD = {0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	// test motion
	BOOST_CHECK_EQUAL(j.pose<double>({2.}), trans2);

	// test motion
	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

BOOST_AUTO_TEST_CASE(JointTest)
{
	using namespace rbd;

	// test operator==
	Joint j1(Joint::RevX, true, "j1");
	Joint j2(Joint::RevX, false, "j2");

	BOOST_CHECK_EQUAL(j1, j1);
	BOOST_CHECK_NE(j1, j2);

	// Test operator!=
	BOOST_CHECK(!(j1 != j1));
	BOOST_CHECK(j1 != j2);

	// Test direction
	BOOST_CHECK_EQUAL(j1.direction(), 1.);
	BOOST_CHECK_EQUAL(j2.direction(), -1.);

	// Test forward (getter)
	BOOST_CHECK_EQUAL(j1.forward(), true);
	BOOST_CHECK_EQUAL(j2.forward(), false);

	// Test forward (setter)
	j1.forward(false);
	BOOST_CHECK_EQUAL(j1.direction(), -1.);
	BOOST_CHECK_EQUAL(j1.forward(), false);

	// sPose
	BOOST_CHECK_THROW(j1.sPose({0., 0.}), std::domain_error);
	BOOST_CHECK_THROW(j1.sPose({}), std::domain_error);
	BOOST_CHECK_NO_THROW(j1.sPose({0.}));
	BOOST_CHECK_EQUAL(j1.sPose({0.}), j1.pose<double>({0.}));

	// sMotion
	BOOST_CHECK_THROW(j1.sMotion({0., 0.}), std::domain_error);
	BOOST_CHECK_THROW(j1.sMotion({}), std::domain_error);
	BOOST_CHECK_NO_THROW(j1.sMotion({0.}));
	BOOST_CHECK_EQUAL(j1.sMotion({0.}), j1.motion({0.}));
}

BOOST_AUTO_TEST_CASE(RevXTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevX, Vector3d::UnitX(), true);
	testRevolute(Joint::RevX, Vector3d::UnitX(), false);
}

BOOST_AUTO_TEST_CASE(RevYTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevY, Vector3d::UnitY(), true);
	testRevolute(Joint::RevY, Vector3d::UnitY(), false);
}

BOOST_AUTO_TEST_CASE(RevZTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevZ, Vector3d::UnitZ(), true);
	testRevolute(Joint::RevZ, Vector3d::UnitZ(), false);
}

BOOST_AUTO_TEST_CASE(PrismXTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatic(Joint::PrismX, Vector3d::UnitX(), true);
	testPrismatic(Joint::PrismX, Vector3d::UnitX(), false);
}

BOOST_AUTO_TEST_CASE(PrismYTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatic(Joint::PrismY, Vector3d::UnitY(), true);
	testPrismatic(Joint::PrismY, Vector3d::UnitY(), false);
}

BOOST_AUTO_TEST_CASE(PrismZTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatic(Joint::PrismZ, Vector3d::UnitZ(), true);
	testPrismatic(Joint::PrismZ, Vector3d::UnitZ(), false);
}

BOOST_AUTO_TEST_CASE(SphericalTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Spherical, true, "sphere");

	// subspace data
	MatrixXd S = MatrixXd::Zero(6,3);
	S.block(0,0,3,3).setIdentity();

	// pose data
	double rotX = constants::pi<double>()/2.;
	double rotY = constants::pi<double>()/4.;
	double rotZ = constants::pi<double>();

	Quaterniond quat = AngleAxisd(rotX, Vector3d::UnitX())*
		AngleAxisd(rotY, Vector3d::UnitY())*AngleAxisd(rotZ, Vector3d::UnitZ());

	std::vector<double> q = {quat.w(), quat.x(), quat.y(), quat.z()};

	PTransformd rot(quat.inverse());

	// motion data
	std::vector<double> alpha;
	Vector3d alphaE = Vector3d::Random();
	for(int i = 0; i < 3; ++i)
		alpha.push_back(alphaE(i));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Spherical);
	BOOST_CHECK_EQUAL(j.params(), 4);
	BOOST_CHECK_EQUAL(j.dof(), 3);
	BOOST_CHECK_EQUAL(j.name(), "sphere");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {1., 0., 0., 0.};
	std::vector<double> zeroD = {0., 0., 0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	// test pose
#ifdef __i386__
	BOOST_CHECK_SMALL((j.pose(q).matrix() - rot.matrix()).array().abs().sum(), TOL);
#else
	BOOST_CHECK_EQUAL(j.pose(q), rot);
#endif

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);

	// test inverse polarity
	j.forward(false);
	BOOST_CHECK_EQUAL(j.motionSubspace(), -S);
#ifdef __i386__
	BOOST_CHECK_SMALL((j.pose(q).matrix() - rot.inv().matrix()).array().abs().sum(), TOL);
#else
	BOOST_CHECK_EQUAL(j.pose(q), rot.inv());
#endif
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), -S*alphaE);
}

BOOST_AUTO_TEST_CASE(PlanarTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Planar, true, "planar");

	// subspace data
	MatrixXd S = MatrixXd::Zero(6,3);
	S.block(2,0,3,3).setIdentity();

	// pose data
	double rotZ = constants::pi<double>()/2.;
	double transX = 4.;
	double transY = 0.3;

	sva::PTransformd trans(RotZ(rotZ), RotZ(rotZ).transpose()*Vector3d(transX, transY, 0.));

	std::vector<double> q = {rotZ, transX, transY};

	// motion data
	std::vector<double> alpha;
	Vector3d alphaE = Vector3d::Random();
	for(int i = 0; i < 3; ++i)
		alpha.push_back(alphaE(i));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Planar);
	BOOST_CHECK_EQUAL(j.params(), 3);
	BOOST_CHECK_EQUAL(j.dof(), 3);
	BOOST_CHECK_EQUAL(j.name(), "planar");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {0., 0., 0.};
	std::vector<double> zeroD = {0., 0., 0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	// test pose
	BOOST_CHECK_EQUAL(j.pose(q), trans);

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);

	// test inverse polarity
	j.forward(false);
	BOOST_CHECK_EQUAL(j.motionSubspace(), -S);
	BOOST_CHECK_EQUAL(j.pose(q), trans.inv());
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), -S*alphaE);
}

BOOST_AUTO_TEST_CASE(CylindricalTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Vector3d axis(Vector3d::Random().normalized());
	Joint j(Joint::Cylindrical, axis, true, "cylindrical");

	// subspace data
	MatrixXd S = MatrixXd::Zero(6,2);
	S.col(0).head<3>() = axis;
	S.col(1).tail<3>() = axis;

	// pose data
	double rot = constants::pi<double>()/2.;
	double trans = 5.;

	sva::PTransformd X(AngleAxisd(-rot, axis).matrix(), axis*trans);

	std::vector<double> q = {rot, trans};

	// motion data
	std::vector<double> alpha;
	Vector2d alphaE = Vector2d::Random();
	for(int i = 0; i < 2; ++i)
		alpha.push_back(alphaE(i));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Cylindrical);
	BOOST_CHECK_EQUAL(j.params(), 2);
	BOOST_CHECK_EQUAL(j.dof(), 2);
	BOOST_CHECK_EQUAL(j.name(), "cylindrical");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {0., 0.};
	std::vector<double> zeroD = {0., 0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	// test pose
	BOOST_CHECK_EQUAL(j.pose(q), X);

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);

	// test inverse polarity
	j.forward(false);
	BOOST_CHECK_EQUAL(j.motionSubspace(), -S);
	BOOST_CHECK_SMALL((j.pose(q).matrix() - X.inv().matrix()).norm(), TOL);
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), -S*alphaE);
}

BOOST_AUTO_TEST_CASE(FreeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Free, true, "free");

	// subspace data
	MatrixXd S = MatrixXd::Identity(6,6);

	// pose data
	double rotX = constants::pi<double>()/2.;
	double rotY = constants::pi<double>()/4.;
	double rotZ = constants::pi<double>();

	Quaterniond quat = AngleAxisd(rotX, Vector3d::UnitX())*
		AngleAxisd(rotY, Vector3d::UnitY())*AngleAxisd(rotZ, Vector3d::UnitZ());
	Vector3d trans = Vector3d::Random();

	std::vector<double> q = {quat.w(), quat.x(), quat.y(), quat.z(),
		trans.x(), trans.y(), trans.z()};

	PTransformd rot(quat.matrix().transpose(), trans);

	// motion data
	std::vector<double> alpha;
	Vector6d alphaE = Vector6d::Random();
	for(int i = 0; i < 6; ++i)
		alpha.push_back(alphaE(i));


	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Free);
	BOOST_CHECK_EQUAL(j.params(), 7);
	BOOST_CHECK_EQUAL(j.dof(), 6);
	BOOST_CHECK_EQUAL(j.name(), "free");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test zero
	std::vector<double> zeroP = {1., 0., 0., 0., 0., 0., 0.};
	std::vector<double> zeroD = {0., 0., 0., 0., 0., 0.};
	std::vector<double> zp = j.zeroParam();
	std::vector<double> zd = j.zeroDof();
	BOOST_CHECK_EQUAL_COLLECTIONS(zp.begin(), zp.end(), zeroP.begin(), zeroP.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(zd.begin(), zd.end(), zeroD.begin(), zeroD.end());

	// test pose
	BOOST_CHECK_SMALL((j.pose(q).matrix() - rot.matrix()).norm(), TOL);

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);

	// test inverse polarity
	j.forward(false);
	BOOST_CHECK_EQUAL(j.motionSubspace(), -S);
	BOOST_CHECK_SMALL((j.pose(q).matrix() - rot.inv().matrix()).norm(), TOL);
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), -S*alphaE);
}

BOOST_AUTO_TEST_CASE(FixedTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	Joint j(Joint::Fixed, true, "fixed");

	// subspace data
	MatrixXd S = MatrixXd::Zero(6,0);

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Fixed);
	BOOST_CHECK_EQUAL(j.params(), 0);
	BOOST_CHECK_EQUAL(j.dof(), 0);
	BOOST_CHECK_EQUAL(j.name(), "fixed");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test pose
	BOOST_CHECK_EQUAL(j.pose<double>({}), PTransformd::Identity());

	// test motion
	BOOST_CHECK_EQUAL(j.motion({}).vector(), Vector6d::Zero());
}
