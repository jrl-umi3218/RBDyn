#include <RBDyn/Body.h>
#include <RBDyn/FK.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <iostream>

int main()
{
  rbd::MultiBodyGraph mbg;

  double mass = 1.;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Vector3d h = Eigen::Vector3d::Zero();

  sva::RBInertiad rbi(mass, h, I);

  rbd::Body b0(rbi, "b0");
  rbd::Body b1(rbi, "b1");
  rbd::Body b2(rbi, "b2");
  rbd::Body b3(rbi, "b3");

  mbg.addBody(b0);
  mbg.addBody(b1);
  mbg.addBody(b2);
  mbg.addBody(b3);

  rbd::Joint j0(rbd::Joint::RevX, true, "j0");
  rbd::Joint j1(rbd::Joint::RevX, true, "j1");
  rbd::Joint j2(rbd::Joint::RevX, true, "j2");

  mbg.addJoint(j0);
  mbg.addJoint(j1);
  mbg.addJoint(j2);

  //  Root     j0       j1     j2
  //  ---- b0 ---- b1 ---- b2 ----b3
  //  Base    RevX   RevX    RevX

  sva::PTransformd to(Eigen::Vector3d(0., 1., 0.));
  sva::PTransformd from(Eigen::Vector3d(0., 0., 0.));

  mbg.linkBodies("b0", from, "b1", from, "j0");
  mbg.linkBodies("b1", to, "b2", from, "j1");
  mbg.linkBodies("b2", to, "b3", from, "j2");

  rbd::MultiBody mb = mbg.makeMultiBody("b0", true);

  rbd::MultiBodyConfig mbc(mb);
  mbc.zero(mb);
  rbd::forwardKinematics(mb, mbc);
  std::cout << mbc.bodyPosW[mb.bodies().size() - 1] << "\n";
  return 0;
}
