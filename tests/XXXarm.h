/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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


/// @return A simple XXX arm with Y as up axis.
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph>
makeXXXarm(bool isFixed=true)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();
	//Vector3d h = Vector3d(0, -0.25, 0);

	RBInertiad rbi(mass, h, I);

	Body b0(rbi, "b0");
	Body b1(rbi, "b1");
	Body b2(rbi, "b2");
	Body b3(rbi, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::RevX, true, "j0");
	Joint j1(Joint::RevX, true, "j1");
	Joint j2(Joint::RevX, true, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	//  Root     j0       j1     j2
	//  ---- b0 ---- b1 ---- b2 ----b3
	//  Base    RevX   RevX    RevX


	PTransformd to(Vector3d(0., 1., 0.));
	PTransformd from(Vector3d(0., 0., 0.));


	mbg.linkBodies("b0", from, "b1", from, "j0");
	mbg.linkBodies("b1", to, "b2", from, "j1");
	mbg.linkBodies("b2", to, "b3", from, "j2");

	MultiBody mb = mbg.makeMultiBody("b0", isFixed);

	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}
