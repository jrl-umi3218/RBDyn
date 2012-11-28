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
#include <SpaceVecAlg>

// RBDyn
#include "Joint.h"

const double TOL = 1e-10;

void testRevolute(rbd::Joint::Type type, const Eigen::Vector3d& axis, bool forward)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, forward, 0, "rev");
	double dir = forward ? 1. : -1;

	// test motion
	Vector6d S;
	S << dir*axis, 0., 0., 0.;

	// test motion
	MotionVec motion(S);

	// test motion
	PTransform rot90(AngleAxisd(-constants::pi<double>()/2., dir*axis).matrix());

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), type);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.id(), 0);
	BOOST_CHECK_EQUAL(j.name(), "rev");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	BOOST_CHECK_EQUAL(j.pose({constants::pi<double>()/2.}), rot90);

	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

void testPrismatique(rbd::Joint::Type type, const Eigen::Vector3d& axis, bool forward)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, forward, 0, "prism");
	double dir = forward ? 1. : -1;

	// test motion
	Vector6d S;
	S << 0., 0., 0., dir*axis;

	// test motion
	MotionVec motion(S);

	// test motion
	PTransform trans2(Vector3d(dir*axis*2.));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), type);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.id(), 0);
	BOOST_CHECK_EQUAL(j.name(), "prism");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test motion
	BOOST_CHECK_EQUAL(j.pose({2.}), trans2);

	// test motion
	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

BOOST_AUTO_TEST_CASE(JointTest)
{
	using namespace rbd;

	// test operator==
	Joint j1(Joint::RevX, true, 0, "j1");
	Joint j2(Joint::RevX, false, 1, "j2");

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
	BOOST_CHECK_EQUAL(j1.sPose({0.}), j1.pose({0.}));

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
	testPrismatique(Joint::PrismX, Vector3d::UnitX(), true);
	testPrismatique(Joint::PrismX, Vector3d::UnitX(), false);
}

BOOST_AUTO_TEST_CASE(PrismYTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatique(Joint::PrismY, Vector3d::UnitY(), true);
	testPrismatique(Joint::PrismY, Vector3d::UnitY(), false);
}

BOOST_AUTO_TEST_CASE(PrismZTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatique(Joint::PrismZ, Vector3d::UnitZ(), true);
	testPrismatique(Joint::PrismZ, Vector3d::UnitZ(), false);
}

BOOST_AUTO_TEST_CASE(SphericalTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Spherical, true, 2, "sphere");

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

	PTransform rot(quat);

	// motion data
	std::vector<double> alpha;
	Vector3d alphaE = Vector3d::Random();
	for(int i = 0; i < 3; ++i)
		alpha.push_back(alphaE(i));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Spherical);
	BOOST_CHECK_EQUAL(j.params(), 4);
	BOOST_CHECK_EQUAL(j.dof(), 3);
	BOOST_CHECK_EQUAL(j.id(), 2);
	BOOST_CHECK_EQUAL(j.name(), "sphere");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test pose
	BOOST_CHECK_EQUAL(j.pose(q), rot);

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);

	// test inverse polarity
	j.forward(false);
	BOOST_CHECK_EQUAL(j.motionSubspace(), -S);
	BOOST_CHECK_EQUAL(j.pose(q), rot.inv());
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), -S*alphaE);

}

BOOST_AUTO_TEST_CASE(FreeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Free, true, 2, "free");

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

	PTransform rot(quat.matrix().transpose(), quat.matrix()*trans);

	// motion data
	std::vector<double> alpha;
	Vector6d alphaE = Vector6d::Random();
	for(int i = 0; i < 6; ++i)
		alpha.push_back(alphaE(i));


	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Free);
	BOOST_CHECK_EQUAL(j.params(), 7);
	BOOST_CHECK_EQUAL(j.dof(), 6);
	BOOST_CHECK_EQUAL(j.id(), 2);
	BOOST_CHECK_EQUAL(j.name(), "free");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

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

	Joint j(Joint::Fixed, true, 2, "fixed");

	// subspace data
	MatrixXd S = MatrixXd::Zero(6,0);

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), Joint::Fixed);
	BOOST_CHECK_EQUAL(j.params(), 0);
	BOOST_CHECK_EQUAL(j.dof(), 0);
	BOOST_CHECK_EQUAL(j.id(), 2);
	BOOST_CHECK_EQUAL(j.name(), "fixed");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test pose
	BOOST_CHECK_EQUAL(j.pose({}), PTransform::Identity());

	// test motion
	BOOST_CHECK_EQUAL(j.motion({}).vector(), Vector6d::Zero());
}
