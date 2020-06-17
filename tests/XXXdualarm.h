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
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig, rbd::MultiBodyGraph> makeXXXdualarm(bool isFixed = true)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  MultiBodyGraph mbg;

  double mass = 1.;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();

  RBInertiad rbi(mass, h, I);

  Body b0(rbi, "b0");
  Body b1(rbi, "b1");
  Body b21(rbi, "b21");
  Body b22(rbi, "b22");
  Body b31(rbi, "b31");
  Body b32(rbi, "b32");

  mbg.addBody(b0);
  mbg.addBody(b1);
  mbg.addBody(b21);
  mbg.addBody(b22);
  mbg.addBody(b31);
  mbg.addBody(b32);

  Joint j0(Joint::RevX, true, "j0");
  Joint j11(Joint::RevX, true, "j11");
  Joint j12(Joint::RevX, true, "j12");
  Joint j21(Joint::RevX, true, "j21");
  Joint j22(Joint::RevX, true, "j22");

  mbg.addJoint(j0);
  mbg.addJoint(j11);
  mbg.addJoint(j12);
  mbg.addJoint(j21);
  mbg.addJoint(j22);

  //                    j11       j21
  //                   -----  b21 ----- b31
  //                  |
  //  Root      j0    |
  //  ----- b0 ----- b1
  //  Base            |
  //                  | j21       j22
  //                   -----  b22 ----- b32

  PTransformd to(Vector3d(0., 1., 0.));
  PTransformd to1(Vector3d(0., 1., 1.));
  PTransformd to2(Vector3d(0., 1., -1.));
  PTransformd from(Vector3d(0., -1., 0.));

  mbg.linkBodies("b0", to, "b1", from, "j0");
  mbg.linkBodies("b1", to1, "b21", from, "j11");
  mbg.linkBodies("b1", to2, "b22", from, "j12");
  mbg.linkBodies("b21", to, "b31", from, "j21");
  mbg.linkBodies("b22", to, "b32", from, "j22");

  MultiBody mb = mbg.makeMultiBody("b0", isFixed);

  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  return std::make_tuple(mb, mbc, mbg);
}
