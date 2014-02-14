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

#pragma once

// includes
// std
#include <tuple>

// RBDyn
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"


std::tuple<int, int> createLeg(rbd::MultiBodyGraph& mbg,
	const Eigen::Vector3d& direction, int parentId,
	int beginBId, int beginJId, const std::string& prefix)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	mbg.addBody({rbi, beginBId + 0, prefix + "LEG0"});
	mbg.addBody({rbi, beginBId + 1, prefix + "LEG1"});
	mbg.addBody({rbi, beginBId + 2, prefix + "LEG2"});
	mbg.addBody({rbi, beginBId + 3, prefix + "LEG3"});
	mbg.addBody({rbi, beginBId + 4, prefix + "LEG4"});
	mbg.addBody({rbi, beginBId + 5, prefix + "LEG5"});

	mbg.addJoint({Joint::RevX, true, beginJId + 0, prefix + "LEGBASE_0"});
	mbg.addJoint({Joint::RevY, true, beginJId + 1, prefix + "LEG0_1"});
	mbg.addJoint({Joint::RevZ, true, beginJId + 2, prefix + "LEG1_2"});
	mbg.addJoint({Joint::RevX, true, beginJId + 3, prefix + "LEG2_3"});
	mbg.addJoint({Joint::RevY, true, beginJId + 4, prefix + "LEG3_5"});
	mbg.addJoint({Joint::RevX, true, beginJId + 5, prefix + "LEG4_5"});

	PTransformd to(direction);
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies(parentId, to, beginBId + 0, from, beginJId + 0);
	mbg.linkBodies(beginBId + 0, to, beginBId + 1, from, beginJId + 1);
	mbg.linkBodies(beginBId + 1, to, beginBId + 2, from, beginJId + 2);
	mbg.linkBodies(beginBId + 2, to, beginBId + 3, from, beginJId + 3);
	mbg.linkBodies(beginBId + 3, to, beginBId + 4, from, beginJId + 4);
	mbg.linkBodies(beginBId + 4, to, beginBId + 5, from, beginJId + 5);

	return std::make_tuple(beginBId + 6, beginJId + 6);
}


std::tuple<int, int> createArm(rbd::MultiBodyGraph& mbg,
	const Eigen::Vector3d& direction, int parentId,
	int beginBId, int beginJId, const std::string& prefix)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	mbg.addBody({rbi, beginBId + 0, prefix + "ARM0"});
	mbg.addBody({rbi, beginBId + 1, prefix + "ARM1"});
	mbg.addBody({rbi, beginBId + 2, prefix + "ARM2"});
	mbg.addBody({rbi, beginBId + 3, prefix + "ARM3"});
	mbg.addBody({rbi, beginBId + 4, prefix + "ARM4"});
	mbg.addBody({rbi, beginBId + 5, prefix + "ARM5"});
	mbg.addBody({rbi, beginBId + 6, prefix + "ARM6"});

	mbg.addJoint({Joint::RevX, true, beginJId + 0, prefix + "ARMBASE_0"});
	mbg.addJoint({Joint::RevY, true, beginJId + 1, prefix + "ARM0_1"});
	mbg.addJoint({Joint::RevZ, true, beginJId + 2, prefix + "ARM1_2"});
	mbg.addJoint({Joint::RevX, true, beginJId + 3, prefix + "ARM2_3"});
	mbg.addJoint({Joint::RevX, true, beginJId + 4, prefix + "ARM3_5"});
	mbg.addJoint({Joint::RevY, true, beginJId + 5, prefix + "ARM4_5"});
	mbg.addJoint({Joint::RevZ, true, beginJId + 6, prefix + "ARM5_6"});

	PTransformd to(direction);
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies(parentId, to, beginBId + 0, from, beginJId + 0);
	mbg.linkBodies(beginBId + 0, to, beginBId + 1, from, beginJId + 1);
	mbg.linkBodies(beginBId + 1, to, beginBId + 2, from, beginJId + 2);
	mbg.linkBodies(beginBId + 2, to, beginBId + 3, from, beginJId + 3);
	mbg.linkBodies(beginBId + 3, to, beginBId + 4, from, beginJId + 4);
	mbg.linkBodies(beginBId + 4, to, beginBId + 5, from, beginJId + 5);
	mbg.linkBodies(beginBId + 5, to, beginBId + 6, from, beginJId + 6);

	return std::make_tuple(beginBId + 7, beginJId + 7);
}

/// @return An simple XYZ arm with Y as up axis.
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph>
makeTree30Dof(bool isFixed=true)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	mbg.addBody({rbi, 0, "BODY0"});
	mbg.addBody({rbi, 1, "BODY1"});
	mbg.addBody({rbi, 2, "TORSO"});
	mbg.addBody({rbi, 3, "HEAD0"});
	mbg.addBody({rbi, 4, "HEAD1"});

	mbg.addJoint({Joint::RevY, true, 0, "BODY0_BODY1"});
	mbg.addJoint({Joint::RevX, true, 1, "BODY1_TORSO"});
	mbg.addJoint({Joint::RevY, true, 2, "TORSO_HEAD0"});
	mbg.addJoint({Joint::RevX, true, 3, "HEAD0_HEAD1"});

	PTransformd to(Vector3d(0., 0.1, 0.));
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies(0, to, 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);
	mbg.linkBodies(3, to, 4, from, 3);

	int nextBId = 5;
	int nextJId = 4;

	// left arm
	std::tie(nextBId, nextJId) = createArm(mbg, Vector3d(-0.1, 0.05, 0.), 2,
		nextBId, nextJId, "L");
	// right arm
	std::tie(nextBId, nextJId) = createArm(mbg, Vector3d(0.1, 0.05, 0.), 2,
		nextBId, nextJId, "R");
	// left leg
	std::tie(nextBId, nextJId) = createLeg(mbg, Vector3d(-0.1, -0.05, 0.), 0,
		nextBId, nextJId, "L");
	// right leg
	std::tie(nextBId, nextJId) = createLeg(mbg, Vector3d(0.1, -0.05, 0.), 0,
		nextBId, nextJId, "R");

	MultiBody mb = mbg.makeMultiBody(0, isFixed);

	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}
