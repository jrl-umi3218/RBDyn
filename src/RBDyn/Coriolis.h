#pragma once

#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/CoM.h"

namespace coriolis
{

using Block = std::array<int, 3>;
using Blocks = std::vector<Block>;

Eigen::Matrix3d skewSymmetricFromVelocity(const Eigen::Vector3d& v);

Eigen::Matrix3d rotationMatrixFromVector(const Eigen::Vector3d& angular);

Eigen::MatrixXd expand(const rbd::Jacobian& jac,
                       const rbd::MultiBody& mb,
                       const Eigen::MatrixXd& jacMat);

void expandAdd(const rbd::Jacobian& jac,
               const rbd::MultiBody& mb,
               const Eigen::MatrixXd& jacMat,
               Eigen::MatrixXd& res);

Blocks compactPath(const rbd::Jacobian& jac,
                   const rbd::MultiBody& mb);

void compactExpandAdd(const Blocks& compactPath,
                      const Eigen::MatrixXd& jacMat,
                      Eigen::MatrixXd& res);

class Coriolis
{
  public:
    Coriolis(const rbd::MultiBody& mb);

    Eigen::MatrixXd coriolis(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

  private:
    std::vector<rbd::Jacobian> jacs_;
    std::vector<Eigen::MatrixXd> jacMats_;
    std::vector<Eigen::MatrixXd> jacDotMats_;
    std::vector<Blocks> compactPaths_;
};

} // namespace coriolis
