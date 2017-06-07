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
#include <vector>

// boost
#define BOOST_TEST_MODULE JointTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"
#include "RBDyn/EulerIntegration.h"

using namespace Eigen;
using namespace sva;
using namespace rbd;
namespace constants = boost::math::constants;


/// @return A robot with a single joint specified by the user
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph>
makeSingleJointRobot(Joint::Type type, const Vector3d& axis = Vector3d::UnitZ())
{
	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();
	RBInertiad rbi(mass, h, I);

	Body b0(rbi, "b0");
	Body b1(rbi, "b1");
	mbg.addBody(b0);
	mbg.addBody(b1);

	Joint j0(type,axis,true,"j0");
	mbg.addJoint(j0);

	PTransformd to(Vector3d(0., 0., 0.));
	PTransformd from(Vector3d(0., 0., 0.));
	mbg.linkBodies("b0", to, "b1", from, "j0");

	MultiBody mb = mbg.makeMultiBody("b0", true);
	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}

std::vector<double> randVec(int size, double rmin, double rmax, bool normed = false)
{
	std::vector<double> v(size, 0);
	Map<VectorXd> r(&v[0], size, 1);
	r = (rmax - rmin)*VectorXd::Random(size).cwiseAbs() + VectorXd::Constant(size, rmin);

	if (normed)
		r.normalize();

	return v;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
randQVA(Joint::Type type)
{
	std::vector<double> q, v, a;
	switch (type)
	{
	case Joint::Rev:
	case Joint::Prism:
		q = randVec(1, -1, 1);
		v = randVec(1, -1, 1);
		a = randVec(1, -1, 1);
		break;
	case Joint::Spherical:
		q = randVec(4, -1, 1, true);
		v = randVec(3, -1, 1);
		a = randVec(3, -1, 1);
		break;
	case Joint::Planar:
		q = randVec(3, -1, 1);
		v = randVec(3, -1, 1);
		a = randVec(3, -1, 1);
		break;
	case Joint::Cylindrical:
		q = randVec(2, -1, 1);
		v = randVec(2, -1, 1);
		a = randVec(2, -1, 1);
		break;
	case Joint::Free:
	{
		q = randVec(4, -1, 1, true);
		auto qt = randVec(3, -1, 1);
		q.insert(q.end(), qt.begin(), qt.end());
		v = randVec(6, -1, 1);
		a = randVec(6, -1, 1);
		break;
	}
	default:
		break;
	}

	return std::make_tuple(q, v, a);
}

void testConstantSpeedIntegration(Joint::Type type,
																	double step,
																	const std::vector<double>& q,
																	const std::vector<double>& v,
																	const std::vector<double>& q_expected)
{
	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeSingleJointRobot(type);

	mbc.q = { {}, q };
	mbc.alpha = { {}, v };
	mbc.alphaD = { {}, std::vector<double>(v.size(), 0) };
	forwardKinematics(mb, mbc);

	eulerIntegration(mb, mbc, step);

	for (size_t i=0; i<q.size(); ++i)
	{
		BOOST_CHECK_CLOSE_FRACTION(q_expected[i], mbc.q[1][i], 1e-8);
	}
}

void testConstantAccelerationIntegration(Joint::Type type,
																				 double step,
																				 const std::vector<double>& q,
																				 const std::vector<double>& v,
																				 const std::vector<double>& a)
{
	MultiBody mb;
	MultiBodyConfig mbc;
	MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeSingleJointRobot(type);
	MultiBodyConfig mbc0(mbc);

	mbc.q = { {}, q };
	mbc.alpha = { {}, v };
	mbc.alphaD = { {}, std::vector<double>(v.size(), 0) };
	mbc0.q = { {}, q };
	mbc0.alpha = { {}, v };
	mbc0.alphaD = { {}, a };

	forwardKinematics(mb, mbc);
	forwardKinematics(mb, mbc0);

	//integrating on the whole time step
	eulerIntegration(mb, mbc0, step);

	//integrating with constant velocity on small time step
	const int N = 2000;
	for (int i = 0; i < N; ++i)
	{
		eulerIntegration(mb, mbc, step/N);
		for (size_t j = 0; j < v.size(); ++j)
			mbc.alpha[1][j] += a[j] * step / N;
		forwardKinematics(mb, mbc);
	}

	for (size_t i=0; i<q.size(); ++i)
	{
		BOOST_CHECK_CLOSE_FRACTION(mbc0.q[1][i], mbc.q[1][i], 1e-3);
	}
}

BOOST_AUTO_TEST_CASE(JointIntegrationTest)
{
	const double pi = constants::pi<double>();
	const double c2 = std::sqrt(2) / 2;

	testConstantSpeedIntegration(Joint::Rev, 1, { 1 }, { 0.5 }, { 1.5 });
	testConstantSpeedIntegration(Joint::Prism, 1, { 1 }, { 0.5 }, { 1.5 });
	testConstantSpeedIntegration(Joint::Spherical, 1, { 1, 0, 0, 0 }, { pi / 2, 0, 0 }, { c2, c2, 0, 0 });
	testConstantSpeedIntegration(Joint::Spherical, 1, { c2, 0, c2, 0 }, { pi / 2, 0, 0 }, { 0.5, 0.5, 0.5, -0.5 });
	//testConstantSpeedIntegration(Joint::Planar, 1, { 1 }, { 0.5 }, { 1.5 });
	testConstantSpeedIntegration(Joint::Cylindrical, 1, { 1, 2 }, { 0.5, 0.25 }, { 1.5, 2.25 });
	testConstantSpeedIntegration(Joint::Free, 1, { 1, 0, 0, 0, 1, 2, 3 }, { pi / 2, 0, 0, 0.5, 0.25, -0.5 }, { c2, c2, 0, 0, 1.5, 2.25, 2.5 });
	testConstantSpeedIntegration(Joint::Free, 1, { c2, 0, c2, 0, 1, 2, 3 }, { pi / 2, 0, 0, 0.5, 0.25, -0.5 }, { 0.5, 0.5, 0.5, -0.5, 0.5, 2.25, 2.5 });

	std::vector<double> q, v, a;
	std::tie(q, v, a) = randQVA(Joint::Rev);
	testConstantAccelerationIntegration(Joint::Rev, 1, q, v, a);
	std::tie(q, v, a) = randQVA(Joint::Prism);
	testConstantAccelerationIntegration(Joint::Prism, 1, q, v, a);
	std::tie(q, v, a) = randQVA(Joint::Spherical);
	testConstantAccelerationIntegration(Joint::Spherical, 0.01, q, v, a);
	//std::tie(q, v, a) = randQVA(Joint::Planar);
	//testConstantAccelerationIntegration(Joint::Planar, 1, q, v, a);
	std::tie(q, v, a) = randQVA(Joint::Cylindrical);
	testConstantAccelerationIntegration(Joint::Cylindrical, 1, q, v, a);
	std::tie(q, v, a) = randQVA(Joint::Free);
	testConstantAccelerationIntegration(Joint::Free, 0.01, q, v, a);
}
