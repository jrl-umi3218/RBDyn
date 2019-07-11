/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <RBDyn/Jacobian.h>

#include "Tree30Dof.h"

#define BOOST_TEST_MODULE Expand
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/test/unit_test.hpp>

void setRandomFreeFlyer(rbd::MultiBodyConfig & mbc)
{
  Eigen::Vector3d axis = Eigen::Vector3d::Random();
  Eigen::AngleAxisd aa(0.5, axis / axis.norm());
  Eigen::Quaterniond qd(aa);
  mbc.q[0][0] = qd.w();
  mbc.q[0][1] = qd.x();
  mbc.q[0][2] = qd.y();
  mbc.q[0][3] = qd.z();
}

BOOST_AUTO_TEST_CASE(ExpandJacobianTest)
{
  std::srand(133757348);

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;

  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  const static int ROUNDS = 1000;

  rbd::Jacobian jac(mb, mb.body(mb.nrBodies() - 1).name());
  rbd::Blocks compact = jac.compactPath(mb);

  for(int i = 0; i < ROUNDS; ++i)
  {
    mbc.zero(mb);

    Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
    mbc.q = rbd::vectorToParam(mb, q);
    setRandomFreeFlyer(mbc);

    rbd::forwardKinematics(mb, mbc);
    rbd::forwardVelocity(mb, mbc);

    Eigen::MatrixXd jacMat = jac.jacobian(mb, mbc);

    Eigen::MatrixXd fullJacMat(6, mb.nrDof());
    jac.fullJacobian(mb, jacMat, fullJacMat);

    Eigen::MatrixXd res = fullJacMat.transpose() * fullJacMat;

    Eigen::MatrixXd product = jacMat.transpose() * jacMat;
    Eigen::MatrixXd fullProduct = jac.expand(mb, product);

    BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

    fullProduct.setZero();
    jac.expandAdd(mb, product, fullProduct);

    BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);

    fullProduct.setZero();
    jac.expandAdd(compact, product, fullProduct);

    BOOST_CHECK_EQUAL((fullProduct - res).norm(), 0);
  }
}
