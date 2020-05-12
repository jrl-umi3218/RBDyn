/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// boost
#define BOOST_TEST_MODULE URDFOutputTest
#include <boost/test/unit_test.hpp>

// RBDyn URDF parser
#include <RBDyn/parsers/urdf.h>

#include <iostream>

// Test utilties
#include "ParsersTestUtils.h"

BOOST_AUTO_TEST_CASE(outputTest)
{
  auto strRobot = rbd::parsers::from_urdf(XYZSarmUrdf);
  auto urdf = rbd::parsers::to_urdf(strRobot);
  std::cout << urdf << std::endl;
  auto strRobot2 = rbd::parsers::from_urdf(urdf);

  BOOST_CHECK_EQUAL(strRobot.mb.nrBodies(), strRobot2.mb.nrBodies());
  BOOST_CHECK_EQUAL(strRobot.mb.nrJoints(), strRobot2.mb.nrJoints());
  BOOST_CHECK_EQUAL(strRobot.mb.nrParams(), strRobot2.mb.nrParams());
  BOOST_CHECK_EQUAL(strRobot.mb.nrDof(), strRobot2.mb.nrDof());

  BOOST_CHECK(std::equal(strRobot.mb.predecessors().begin(), strRobot.mb.predecessors().end(),
                         strRobot2.mb.predecessors().begin()));
  BOOST_CHECK(
      std::equal(strRobot.mb.successors().begin(), strRobot.mb.successors().end(), strRobot2.mb.successors().begin()));
  BOOST_CHECK(std::equal(strRobot.mb.parents().begin(), strRobot.mb.parents().end(), strRobot2.mb.parents().begin()));
  BOOST_CHECK(
      std::equal(strRobot.mb.transforms().begin(), strRobot.mb.transforms().end(), strRobot2.mb.transforms().begin()));

  BOOST_CHECK(std::equal(strRobot.limits.lower.begin(), strRobot.limits.lower.end(), strRobot2.limits.lower.begin()));
  BOOST_CHECK(std::equal(strRobot.limits.upper.begin(), strRobot.limits.upper.end(), strRobot2.limits.upper.begin()));
  BOOST_CHECK(
      std::equal(strRobot.limits.velocity.begin(), strRobot.limits.velocity.end(), strRobot2.limits.velocity.begin()));
  BOOST_CHECK(
      std::equal(strRobot.limits.torque.begin(), strRobot.limits.torque.end(), strRobot2.limits.torque.begin()));

  const auto & str_visuals = strRobot.visual;
  const auto & str2_visuals = strRobot2.visual;
  BOOST_CHECK_EQUAL(str2_visuals.size(), str_visuals.size());
  for(const auto & g : str2_visuals)
  {
    BOOST_CHECK_EQUAL(g.second.size(), str_visuals.at(g.first).size());
  }

  const auto & str_collision = strRobot.collision;
  const auto & str2_collision = strRobot2.collision;
  BOOST_CHECK_EQUAL(str2_collision.size(), str_collision.size());
  for(const auto & g : str2_collision)
  {
    BOOST_CHECK_EQUAL(g.second.size(), str_collision.at(g.first).size());
  }

  for(const auto & body : strRobot.mb.bodies())
  {
    BOOST_CHECK(std::equal(strRobot.visual[body.name()].begin(), strRobot.visual[body.name()].end(),
                           strRobot2.visual[body.name()].begin()));
    BOOST_CHECK(std::equal(strRobot.collision[body.name()].begin(), strRobot.collision[body.name()].end(),
                           strRobot2.collision[body.name()].begin()));
  }

  for(int i = 0; i < strRobot.mb.nrBodies(); ++i)
  {
    const auto & b1 = strRobot.mb.body(i);
    const auto & b2 = strRobot2.mb.body(i);

    BOOST_CHECK_EQUAL(b1.name(), b2.name());

    BOOST_CHECK_EQUAL(b1.inertia().mass(), b2.inertia().mass());
    BOOST_CHECK_EQUAL(b1.inertia().momentum(), b2.inertia().momentum());
    BOOST_CHECK_SMALL((b1.inertia().inertia() - b2.inertia().inertia()).norm(), TOL);
  }

  for(int i = 0; i < strRobot.mb.nrJoints(); ++i)
  {
    const auto & j1 = strRobot.mb.joint(i);
    const auto & j2 = strRobot2.mb.joint(i);

    BOOST_CHECK_EQUAL(j1.name(), j2.name());
    BOOST_CHECK_EQUAL(j1.type(), j2.type());
    BOOST_CHECK_EQUAL(j1.direction(), j2.direction());
    BOOST_CHECK_EQUAL(j1.motionSubspace(), j2.motionSubspace());
  }
}
