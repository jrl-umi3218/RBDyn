#include "XXXarm.h"
#include "XYZarm.h"
#include "Tree30Dof.h"
#include <RBDyn/Coriolis.h>

#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/EulerIntegration.h>

#include <iostream>

int main()
{
  srand(time(NULL));

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;

  std::tie(mb, mbc, mbg) = makeTree30Dof(false);

  const static int ROUNDS = 1000;

  std::vector<double> errors;

  rbd::Jacobian jac(mb, mb.body(mb.nrBodies() - 1).name());

  for(int i = 0; i < ROUNDS; ++i)
  {
    mbc.zero(mb);

    Eigen::VectorXd q = Eigen::VectorXd::Random(mb.nrParams());
    mbc.q = rbd::sVectorToParam(mb, q);

    for(auto& q : mbc.q)
    {
      if(q.size() == 7)
      {
        Eigen::Vector3d axis = Eigen::Vector3d::Random();
        Eigen::AngleAxisd aa(0.5, axis/axis.norm());
        Eigen::Quaterniond qd(aa);
        q[0] = qd.w();
        q[1] = qd.x();
        q[2] = qd.y();
        q[3] = qd.z();
      }
    }

    rbd::forwardKinematics(mb, mbc);
    rbd::forwardVelocity(mb, mbc);

    Eigen::MatrixXd jacMat = jac.jacobian(mb, mbc);

    Eigen::MatrixXd fullJacMat(6, mb.nrDof());
    jac.fullJacobian(mb, jacMat, fullJacMat);

    Eigen::MatrixXd res = fullJacMat.transpose()*fullJacMat;

    Eigen::MatrixXd product = jacMat.transpose()*jacMat;
    Eigen::MatrixXd fullProduct = rbd::expand(jac, mb, product);

    errors.push_back((fullProduct - res).norm());

  }

  auto res = rbd::compactPath(jac, mb);
  for(const auto& pair : res)
  {
    std::cout << "block " << pair[0] << "," << pair[1] << "," << pair[2] << std::endl;
  }

  double mean = 0, max = 0;

  for(auto err : errors)
  {
    mean += err;
    max = std::max(max, err);
  }
  mean /= ROUNDS;

  std::cout << "Over " << ROUNDS << " rounds " << errors.size() << std::endl;
  std::cout << "Mean error expand / full : " << mean << std::endl;
  std::cout << "Max error expand / full : " << max << std::endl;

  return 0;
}
