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
#include <iostream>

// associated header
#include "RBDyn/IS.h"

// includes
// sva
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/util.hh"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Jacobian.h"

namespace rbd
{
using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

InverseStatics::InverseStatics(const MultiBody& mb)
    : f_(mb.nrBodies()),
      df_(mb.nrBodies()),
      jointTorqueDiff_(mb.nrJoints()),
      jacW_(mb.nrBodies()),
      fullJac_(6, mb.nrDof()),
      jacobianSizeHasBeenSet_(false)
{
  fullJac_.setZero();
  for (size_t i = 0; i < static_cast<size_t>(mb.nrBodies()); ++i)
  {
    jacW_[i] = Jacobian(mb, mb.body(static_cast<int>(i)).name());
    df_[i] = MatrixXd::Zero(6, mb.nrDof());
  }

  for (size_t i = 0; i < static_cast<size_t>(mb.nrJoints()); ++i)
  {
    jointTorqueDiff_[i].resize(mb.joint(static_cast<int>(i)).dof(), mb.nrDof());
    jointTorqueDiff_[i].setZero();
  }
}

void InverseStatics::setJacobianSize(
    const MultiBody& mb,
    const MultiBodyConfig& mbc,
    const std::vector<Eigen::MatrixXd>& jacMomentsAndForces)
{
  const std::vector<Body>& bodies = mb.bodies();

  long nColsWanted = 0;
  for (std::size_t i = 0; i < bodies.size(); ++i)
  {
    if(jacMomentsAndForces[i].cols() > nColsWanted)
    {
      nColsWanted = jacMomentsAndForces[i].cols();
    }
  }
  for (std::size_t i = 0; i < bodies.size(); ++i)
  {
    if(nColsWanted > df_[i].cols())
    {
      df_[i].resize(6,nColsWanted);
    }
    df_[i].setZero();
    jointTorqueDiff_[i].resize(mbc.motionSubspace[i].cols(), df_[i].cols());
    jointTorqueDiff_[i].setZero();
  }
  jacobianSizeHasBeenSet_ = true;
}

void InverseStatics::inverseStatics(const MultiBody& mb, MultiBodyConfig& mbc)
{
  const std::vector<Body>& bodies = mb.bodies();
  const std::vector<Joint>& joints = mb.joints();
  const std::vector<int>& pred = mb.predecessors();

  sva::MotionVecd a_0(Vector3d::Zero(), mbc.gravity);

  for (std::size_t i = 0; i < bodies.size(); ++i)
  {
    mbc.bodyAccB[i] = mbc.bodyPosW[i] * a_0;
    f_[i] = bodies[i].inertia() * mbc.bodyAccB[i] -
            mbc.bodyPosW[i].dualMul(mbc.force[i]);
  }

  for (int i = static_cast<int>(joints.size()) - 1; i >= 0; --i)
  {
    // jointTorque is a vector<vector<double>> thus it is necessary to use
    // Eigen::Map to set a vector of elements at once
    // This is identical to do that:
    //
    //    $for (int j = 0; j < joints[i].dof(); ++j)
    //    $  mbc.jointTorque[i][j] = mbc.motionSubspace[i].col(j).transpose() *
    //    f_[i].vector();
    //
    VectorXd::Map(mbc.jointTorque[i].data(), joints[i].dof()) =
        f_[i].vector().transpose() * mbc.motionSubspace[i];

    if (pred[i] != -1)
      f_[pred[i]] += mbc.parentToSon[i].transMul(f_[i]);
  }
}

void InverseStatics::computeTorqueJacobianJoint(
    const MultiBody& mb, MultiBodyConfig& mbc,
    const std::vector<MatrixXd>& jacMomentsAndForces)
{
  assert(jacMomentsAndForces.size() == static_cast<size_t>(mb.nrBodies()));

  auto transMat = [](const sva::PTransformd& T)
  {
    Eigen::Matrix6d res;
    Eigen::Matrix3d Rt = T.rotation().transpose();
    res.block(3, 0, 3, 3).setZero();
    res.block(0, 0, 3, 3) = Rt;
    res.block(0, 3, 3, 3) = vector3ToCrossMatrix(T.translation()) * Rt;
    res.block(3, 3, 3, 3) = Rt;
    return res;
  };

  if(!jacobianSizeHasBeenSet_)
    setJacobianSize(mb, mbc, jacMomentsAndForces);

  const std::vector<Body>& bodies = mb.bodies();
  const std::vector<int>& pred = mb.predecessors();
  const std::vector<Joint>& joints = mb.joints();

  sva::MotionVecd a_0(Vector3d::Zero(), mbc.gravity);

  Matrix6d M;
  Matrix6d N;

  Vector3d aC = a_0.angular();
  Vector3d aF = a_0.linear();
  Matrix3d hatAF, hatAC;
  hatAF = vector3ToCrossMatrix(aF);
  hatAC = vector3ToCrossMatrix(aC);

  for (std::size_t i = 0; i < bodies.size(); ++i)
  {
    df_[i].setZero();
    M.setZero();
    N.setZero();
    // Complete the previously computed jacobian to a full jacobian
    jacW_[i].fullJacobian(mb, jacW_[i].jacobian(mb, mbc), fullJac_);

    mbc.bodyAccB[i] = mbc.bodyPosW[i] * a_0;

    Matrix3d& RW = mbc.bodyPosW[i].rotation();
    Vector3d& tW = mbc.bodyPosW[i].translation();
    Vector3d& fC = mbc.force[i].couple();
    Vector3d& fF = mbc.force[i].force();
    Matrix3d hatFC, hatFF, hathattaC, hathattfF, hattW;
    hatFF = vector3ToCrossMatrix(fF);
    hatFC = vector3ToCrossMatrix(fC);
    hattW = vector3ToCrossMatrix(tW);
    Vector3d hattWaC = hattW * aC;
    Vector3d hattWfF = hattW * fF;
    hathattaC = vector3ToCrossMatrix(hattWaC);
    hathattfF = vector3ToCrossMatrix(hattWfF);

    f_[i] = bodies[i].inertia() * mbc.bodyAccB[i] -
            mbc.bodyPosW[i].dualMul(mbc.force[i]);

    M.block(0, 0, 3, 3) = RW * hatAC;
    M.block(3, 0, 3, 3) = RW * (-hathattaC + hatAF);
    M.block(3, 3, 3, 3) = RW * hatAC;

    N.block(0, 0, 3, 3) = RW * (hatFC - hathattfF);
    N.block(0, 3, 3, 3) = RW * hatFF;
    N.block(3, 0, 3, 3) = RW * hatFF;

    df_[i].block(0,0,fullJac_.rows(), fullJac_.cols()) = (bodies[i].inertia().matrix() * M - N) * fullJac_;

    if (jacMomentsAndForces[i].cols() > 0)
    {
      df_[i] += mbc.bodyPosW[i].dualMatrix() * jacMomentsAndForces[i];
    }

  }

  for (int i = static_cast<int>(joints.size()) - 1; i >= 0; --i)
  {
    jointTorqueDiff_[i] = mbc.motionSubspace[i].transpose() * df_[i];

    if (pred[i] != -1)
    {
      Matrix6d transPtS = transMat(mbc.parentToSon[i]);

      f_[pred[i]] += mbc.parentToSon[i].transMul(f_[i]);
      df_[pred[i]] += transPtS * df_[i];

      Matrix3d& R = mbc.jointConfig[i].rotation();
      Vector3d& t = mbc.jointConfig[i].translation();
      Vector3d RfC = R.transpose() * f_[i].couple();
      Vector3d RfF = R.transpose() * f_[i].force();
      Matrix3d hatRfC = vector3ToCrossMatrix(RfC);
      Matrix3d hatRfF = vector3ToCrossMatrix(RfF);
      Matrix6d MJ;

      MJ.block(0, 0, 3, 3) =
          hatRfC + RfF * t.transpose() - RfF.dot(t) * Matrix3d::Identity();
      MJ.block(0, 3, 3, 3) = -hatRfF;
      MJ.block(3, 0, 3, 3) = hatRfF;
      MJ.block(3, 3, 3, 3).setZero();

      df_[pred[i]].block(0, mb.jointPosInDof(i), 6, joints[i].dof()) -=
          transMat(mb.transforms()[i]) * MJ * mbc.motionSubspace[i];
    }
  }
}

void InverseStatics::computeTorqueJacobianJoint(const MultiBody& mb,
                                                MultiBodyConfig& mbc)
{
  std::vector<MatrixXd> jacMandF;
  for (int i = 0; i < mb.nrJoints(); ++i) jacMandF.push_back(MatrixXd(0, 0));

  computeTorqueJacobianJoint(mb, mbc, jacMandF);
}

void printMBC(const MultiBody& mb, const MultiBodyConfig& mbc)
{
  std::cout << "mb.bodies() = " << mb.bodies() << std::endl;
  std::cout << "mb.joints() = " << mb.joints() << std::endl;
  std::cout << "mb.predecessors() = " << mb.predecessors() << std::endl;
  std::cout << "mbc.gravity = " << mbc.gravity << std::endl;
  std::cout << "mbc.parentToSon = \n" << mbc.parentToSon << std::endl;
  std::cout << "mbc.bodyPosW = \n" << mbc.bodyPosW << std::endl;
  std::cout << "mbc.force = \n" << mbc.force << std::endl;
  std::cout << "mbc.motionSubspace = \n" << mbc.motionSubspace << std::endl;
}

void InverseStatics::sInverseStatics(const MultiBody& mb, MultiBodyConfig& mbc)
{
  checkMatchAlphaD(mb, mbc);
  checkMatchForce(mb, mbc);
  checkMatchJointConf(mb, mbc);
  checkMatchJointVelocity(mb, mbc);
  checkMatchBodyPos(mb, mbc);
  checkMatchParentToSon(mb, mbc);
  checkMatchBodyVel(mb, mbc);
  checkMatchMotionSubspace(mb, mbc);

  checkMatchBodyAcc(mb, mbc);
  checkMatchJointTorque(mb, mbc);

  inverseStatics(mb, mbc);
}

}  // namespace rbd
