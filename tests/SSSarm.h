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


/// @return An simple SSS arm with Y as up axis.
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph>
makeSSSarm(bool isFixed=true)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

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


	PTransformd to(Vector3d(0., 0.5, 0.));
	PTransformd from(Vector3d(0., -0.5, 0.));

	mbg.linkBodies(0, PTransformd::Identity(), 1, from, 0);
	mbg.linkBodies(1, to, 2, from, 1);
	mbg.linkBodies(2, to, 3, from, 2);

	MultiBody mb = mbg.makeMultiBody(0, isFixed);

	MultiBodyConfig mbc(mb);
	mbc.zero(mb);

	return std::make_tuple(mb, mbc, mbg);
}
