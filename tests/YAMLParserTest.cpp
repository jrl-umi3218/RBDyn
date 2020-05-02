/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#define BOOST_TEST_MODULE YAMLParserTest

#include <RBDyn/parsers/yaml.h>

#include <boost/test/unit_test.hpp>

// Test utilties
#include "ParsersTestUtils.h"

BOOST_AUTO_TEST_CASE(loadTest)
{
  auto cppRobot = createRobot();
  auto strRobot = rbd::parsers::from_yaml(XYZSarmYaml);

  BOOST_CHECK_EQUAL(cppRobot.mb.nrBodies(), strRobot.mb.nrBodies());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrJoints(), strRobot.mb.nrJoints());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrParams(), strRobot.mb.nrParams());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrDof(), strRobot.mb.nrDof());

  BOOST_CHECK(std::equal(cppRobot.mb.predecessors().begin(), cppRobot.mb.predecessors().end(),
                         strRobot.mb.predecessors().begin()));
  BOOST_CHECK(
      std::equal(cppRobot.mb.successors().begin(), cppRobot.mb.successors().end(), strRobot.mb.successors().begin()));
  BOOST_CHECK(std::equal(cppRobot.mb.parents().begin(), cppRobot.mb.parents().end(), strRobot.mb.parents().begin()));
  BOOST_CHECK(
      std::equal(cppRobot.mb.transforms().begin(), cppRobot.mb.transforms().end(), strRobot.mb.transforms().begin()));

  BOOST_CHECK(std::equal(cppRobot.limits.lower.begin(), cppRobot.limits.lower.end(), strRobot.limits.lower.begin()));
  BOOST_CHECK(std::equal(cppRobot.limits.upper.begin(), cppRobot.limits.upper.end(), strRobot.limits.upper.begin()));
  BOOST_CHECK(
      std::equal(cppRobot.limits.velocity.begin(), cppRobot.limits.velocity.end(), strRobot.limits.velocity.begin()));
  BOOST_CHECK(std::equal(cppRobot.limits.torque.begin(), cppRobot.limits.torque.end(), strRobot.limits.torque.begin()));

  for(int i = 0; i < cppRobot.mb.nrBodies(); ++i)
  {
    const auto & b1 = cppRobot.mb.body(i);
    const auto & b2 = strRobot.mb.body(i);

    BOOST_CHECK_EQUAL(b1.name(), b2.name());

    BOOST_CHECK_EQUAL(b1.inertia().mass(), b2.inertia().mass());
    BOOST_CHECK_EQUAL(b1.inertia().momentum(), b2.inertia().momentum());
    BOOST_CHECK_SMALL((b1.inertia().inertia() - b2.inertia().inertia()).norm(), TOL);
  }

  for(int i = 0; i < cppRobot.mb.nrJoints(); ++i)
  {
    const auto & j1 = cppRobot.mb.joint(i);
    const auto & j2 = strRobot.mb.joint(i);

    BOOST_CHECK_EQUAL(j1.name(), j2.name());
    BOOST_CHECK_EQUAL(j1.type(), j2.type());
    BOOST_CHECK_EQUAL(j1.direction(), j2.direction());
    BOOST_CHECK_EQUAL(j1.motionSubspace(), j2.motionSubspace());
  }
}

BOOST_AUTO_TEST_CASE(visualTest)
{
  auto cppRobot = createRobot();
  auto strRobot = rbd::parsers::from_yaml(XYZSarmYaml);
  const auto & cpp_geometries = cppRobot.visual;
  const auto & str_geometries = strRobot.visual;

  BOOST_CHECK_EQUAL(str_geometries.size(), cpp_geometries.size());
  for(const auto & g : str_geometries)
  {
    BOOST_CHECK_EQUAL(g.second.size(), cpp_geometries.at(g.first).size());
  }

  for(const auto & body : cppRobot.mb.bodies())
  {
    BOOST_CHECK(std::equal(strRobot.visual[body.name()].begin(), strRobot.visual[body.name()].end(),
                           cppRobot.visual[body.name()].begin()));
  }
}
