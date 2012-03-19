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
#define BOOST_TEST_MODULE BodyTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg>

// RBDyn
#include "Joint.h"

void testRevolute(rbd::Joint::Type type, const Eigen::Vector3d& axis)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, 0, "rev");

	// test motion
	Vector6d S;
	S << axis, 0., 0., 0.;

	// test motion
	MotionVec motion(S);

	// test motion
	PTransform rot90(AngleAxisd(-constants::pi<double>()/2., axis).matrix());

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), type);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.id(), 0);
	BOOST_CHECK_EQUAL(j.name(), "rev");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	BOOST_CHECK_EQUAL(j.pose({constants::pi<double>()/2.}).rotation(),
		rot90.rotation());
	BOOST_CHECK_EQUAL(j.pose({constants::pi<double>()/2.}).translation(),
		rot90.translation());

	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

void testPrismatique(rbd::Joint::Type type, const Eigen::Vector3d& axis)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(type, 0, "prism");

	// test motion
	Vector6d S;
	S << 0., 0., 0., axis;

	// test motion
	MotionVec motion(S);

	// test motion
	PTransform trans2(Vector3d(axis*2.));

	// test accessor
	BOOST_CHECK_EQUAL(j.type(), type);
	BOOST_CHECK_EQUAL(j.params(), 1);
	BOOST_CHECK_EQUAL(j.dof(), 1);
	BOOST_CHECK_EQUAL(j.id(), 0);
	BOOST_CHECK_EQUAL(j.name(), "prism");
	BOOST_CHECK_EQUAL(j.motionSubspace(), S);

	// test motion
	BOOST_CHECK_EQUAL(j.pose({2.}).rotation(),
		trans2.rotation());
	BOOST_CHECK_EQUAL(j.pose({2.}).translation(),
		trans2.translation());

	// test motion
	BOOST_CHECK_EQUAL(j.motion({2.}).vector(), (2.*motion).vector());
}

BOOST_AUTO_TEST_CASE(JointTest)
{
	using namespace rbd;

	// test operator==
	Joint j1(Joint::RevX, 0, "j1");
	Joint j2(Joint::RevX, 1, "j2");

	BOOST_CHECK_EQUAL(j1, j1);
	BOOST_CHECK_NE(j1, j2);

	// Test operator!=
	BOOST_CHECK(!(j1 != j1));
	BOOST_CHECK(j1 != j2);
}

BOOST_AUTO_TEST_CASE(RevXTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevX, Vector3d::UnitX());
}

BOOST_AUTO_TEST_CASE(RevYTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevY, Vector3d::UnitY());
}

BOOST_AUTO_TEST_CASE(RevZTest)
{
	using namespace Eigen;
	using namespace rbd;
	testRevolute(Joint::RevZ, Vector3d::UnitZ());
}

BOOST_AUTO_TEST_CASE(PrismXTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatique(Joint::PrismX, Vector3d::UnitX());
}

BOOST_AUTO_TEST_CASE(PrismYTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatique(Joint::PrismY, Vector3d::UnitY());
}

BOOST_AUTO_TEST_CASE(PrismZTest)
{
	using namespace Eigen;
	using namespace rbd;
	testPrismatique(Joint::PrismZ, Vector3d::UnitZ());
}

BOOST_AUTO_TEST_CASE(SphericalTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Spherical, 2, "sphere");

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
	BOOST_CHECK_EQUAL(j.pose(q).rotation(),
		rot.rotation());
	BOOST_CHECK_EQUAL(j.pose(q).translation(),
		rot.translation());

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);
}

BOOST_AUTO_TEST_CASE(FreeTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	Joint j(Joint::Free, 2, "free");

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

	PTransform rot(quat, quat.matrix()*trans);

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
	BOOST_CHECK_EQUAL(j.pose(q).rotation(),
		rot.rotation());
	BOOST_CHECK_EQUAL(j.pose(q).translation(),
		rot.translation());

	// test motion
	BOOST_CHECK_EQUAL(j.motion(alpha).vector(), S*alphaE);
}

BOOST_AUTO_TEST_CASE(FixedTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	Joint j(Joint::Fixed, 2, "fixed");

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
	BOOST_CHECK_EQUAL(j.pose({}).rotation(),
		Matrix3d::Identity());
	BOOST_CHECK_EQUAL(j.pose({}).translation(),
		Vector3d::Zero());

	// test motion
	BOOST_CHECK_EQUAL(j.motion({}).vector(), Vector6d::Zero());
}
