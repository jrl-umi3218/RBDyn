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

  rbd::MultiBody mbt;
  rbd::MultiBodyConfig mbtc;
  rbd::MultiBodyGraph mbtg;

  std::tie(mbt, mbtc, mbtg) = makeTree30Dof(false);

  const static int ROUNDS = 1000;

  std::vector<double> errors;
  std::vector<double> diff_errors;

  for(int i = 0; i < ROUNDS; ++i)
  {
    mbc.zero(mb);
    mbtc.zero(mbt);

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

    mbtc.q = mbc.q;

    rbd::forwardKinematics(mb, mbc);
    rbd::forwardVelocity(mb, mbc);

    rbd::forwardKinematics(mbt, mbtc);
    rbd::forwardVelocity(mbt, mbtc);

    rbd::ForwardDynamics fd(mbt);
    fd.computeC(mbt, mbtc);
    Eigen::VectorXd gravity = fd.C();

    Eigen::VectorXd qd = Eigen::VectorXd::Random(mb.nrDof());
    mbc.alpha = rbd::sVectorToDof(mb, qd);
    mbtc.alpha = mbc.alpha;

    rbd::forwardVelocity(mb, mbc);
    rbd::forwardVelocity(mbt, mbtc);

    rbd::Coriolis coriolis(mb);
    Eigen::MatrixXd C = coriolis.coriolis(mb, mbc);

    fd.computeC(mbt, mbtc);
    Eigen::MatrixXd N = fd.C();
    errors.push_back((C*qd + gravity - N).norm());

    fd.computeH(mbt, mbtc);
    Eigen::MatrixXd m1 = fd.H();

    double dt = 1e-6;

    rbd::sEulerIntegration(mbt, mbtc, dt);

    rbd::forwardKinematics(mbt, mbtc);
    rbd::forwardVelocity(mbt, mbtc);
    fd.computeH(mbt, mbtc);

    Eigen::MatrixXd m2 = fd.H();

    Eigen::MatrixXd diff = (m2 - m1)/dt - (C+C.transpose());

    diff_errors.push_back(diff.array().abs().maxCoeff());
  }

  double mean = 0, max = 0;

  for(auto err : errors)
  {
    mean += err;
    max = std::max(max, err);
  }
  mean /= ROUNDS;

  std::cout << "Mean error coriolis / RBDyn : " << mean << std::endl;
  std::cout << "Max error coriolis / RBDyn : " << max << std::endl;

  mean = 0;
  max = 0;
  for(auto err : diff_errors)
  {
    mean += err;
    max = std::max(max, err);
  }
  mean /= ROUNDS;

  std::cout << "Mean error skew-symmetry : " << mean << std::endl;
  std::cout << "Max error skew-symmetry : " << max << std::endl;

  return 0;
}
