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

#pragma once

// includes
// std
#include <tuple>

// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"


void createLeg(rbd::MultiBodyGraph& mbg,
	const Eigen::Vector3d& direction,
	const std::string& parentName,
	const std::string& prefix)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	mbg.addBody({rbi, prefix + "LEG0"});
	mbg.addBody({rbi, prefix + "LEG1"});
	mbg.addBody({rbi, prefix + "LEG2"});
	mbg.addBody({rbi, prefix + "LEG3"});
	mbg.addBody({rbi, prefix + "LEG4"});
	mbg.addBody({rbi, prefix + "LEG5"});

	mbg.addJoint({Joint::RevX, true, prefix + "LEGBASE_0"});
	mbg.addJoint({Joint::RevY, true, prefix + "LEG0_1"});
	mbg.addJoint({Joint::RevZ, true, prefix + "LEG1_2"});
	mbg.addJoint({Joint::RevX, true, prefix + "LEG2_3"});
	mbg.addJoint({Joint::RevY, true, prefix + "LEG3_5"});
	mbg.addJoint({Joint::RevX, true, prefix + "LEG4_5"});

	PTransformd to(direction);
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies(parentName, to, prefix + "LEG0", from, prefix + "LEGBASE_0");
	mbg.linkBodies(prefix + "LEG0", to, prefix + "LEG1", from, prefix + "LEG0_1");
	mbg.linkBodies(prefix + "LEG1", to, prefix + "LEG2", from, prefix + "LEG1_2");
	mbg.linkBodies(prefix + "LEG2", to, prefix + "LEG3", from, prefix + "LEG2_3");
	mbg.linkBodies(prefix + "LEG3", to, prefix + "LEG4", from, prefix + "LEG3_5");
	mbg.linkBodies(prefix + "LEG4", to, prefix + "LEG5", from, prefix + "LEG4_5");
}


void createArm(rbd::MultiBodyGraph& mbg,
	const Eigen::Vector3d& direction,
	const std::string& parentName,
	const std::string& prefix)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	mbg.addBody({rbi, prefix + "ARM0"});
	mbg.addBody({rbi, prefix + "ARM1"});
	mbg.addBody({rbi, prefix + "ARM2"});
	mbg.addBody({rbi, prefix + "ARM3"});
	mbg.addBody({rbi, prefix + "ARM4"});
	mbg.addBody({rbi, prefix + "ARM5"});
	mbg.addBody({rbi, prefix + "ARM6"});

	mbg.addJoint({Joint::RevX, true, prefix + "ARMBASE_0"});
	mbg.addJoint({Joint::RevY, true, prefix + "ARM0_1"});
	mbg.addJoint({Joint::RevZ, true, prefix + "ARM1_2"});
	mbg.addJoint({Joint::RevX, true, prefix + "ARM2_3"});
	mbg.addJoint({Joint::RevX, true, prefix + "ARM3_5"});
	mbg.addJoint({Joint::RevY, true, prefix + "ARM4_5"});
	mbg.addJoint({Joint::RevZ, true, prefix + "ARM5_6"});

	PTransformd to(direction);
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies(parentName, to, prefix + "ARM0", from, prefix + "ARMBASE_0");
	mbg.linkBodies(prefix + "ARM0", to, prefix + "ARM1", from, prefix + "ARM0_1");
	mbg.linkBodies(prefix + "ARM1", to, prefix + "ARM2", from, prefix + "ARM1_2");
	mbg.linkBodies(prefix + "ARM2", to, prefix + "ARM3", from, prefix + "ARM2_3");
	mbg.linkBodies(prefix + "ARM3", to, prefix + "ARM4", from, prefix + "ARM3_5");
	mbg.linkBodies(prefix + "ARM4", to, prefix + "ARM5", from, prefix + "ARM4_5");
	mbg.linkBodies(prefix + "ARM5", to, prefix + "ARM6", from, prefix + "ARM5_6");
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

	mbg.addBody({rbi, "BODY0"});
	mbg.addBody({rbi, "BODY1"});
	mbg.addBody({rbi, "TORSO"});
	mbg.addBody({rbi, "HEAD0"});
	mbg.addBody({rbi, "HEAD1"});

	mbg.addJoint({Joint::RevY, true, "BODY0_BODY1"});
	mbg.addJoint({Joint::RevX, true, "BODY1_TORSO"});
	mbg.addJoint({Joint::RevY, true, "TORSO_HEAD0"});
	mbg.addJoint({Joint::RevX, true, "HEAD0_HEAD1"});

	PTransformd to(Vector3d(0., 0.1, 0.));
	PTransformd from(PTransformd::Identity());

	mbg.linkBodies("BODY0", to, "BODY1", from, "BODY0_BODY1");
	mbg.linkBodies("BODY1", to, "TORSO", from, "BODY1_TORSO");
	mbg.linkBodies("TORSO", to, "HEAD0", from, "TORSO_HEAD0");
	mbg.linkBodies("HEAD1", to, "HEAD1", from, "HEAD0_HEAD1");


	// left arm
	createArm(mbg, Vector3d(-0.1, 0.05, 0.), "TORSO", "L");
	// right arm
	createArm(mbg, Vector3d(0.1, 0.05, 0.), "TORSO", "R");
	// left leg
	createLeg(mbg, Vector3d(-0.1, -0.05, 0.), "BODY0", "L");
	// right leg
	createLeg(mbg, Vector3d(0.1, -0.05, 0.), "BODY0", "R");

	MultiBody mb = mbg.makeMultiBody("BODY0", isFixed);

	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}
