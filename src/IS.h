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
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

#include "Jacobian.h"

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
  * Inverse Dynamics algorithm.
  */
class RBDYN_DLLAPI InverseStatics
{
 public:
  InverseStatics() {}
  /// @param mb MultiBody associated with this algorithm.
  InverseStatics(const MultiBody& mb);

  void setJacobianSize(const MultiBody& mb, const MultiBodyConfig& mbc,
                       const std::vector<Eigen::MatrixXd>& jacMomentsAndForces);

  /**
    * Compute the inverse statics.
    * @param mb MultiBody used has model.
    * @param mbc Uses force, parentToSon, bodyPosW, motionSubspace
    * and gravity.
    * Fills bodyAccB and jointTorque.
    */
  void inverseStatics(const MultiBody& mb, MultiBodyConfig& mbc);

  /**
    * Compute the derivatives of the torques calculated by the
    * inverse statics
    * w.r.t. q and forces
    * WARNING: This computes the derivative of the torques w.r.t some fictitious
    * forces applied on each body at the point (0,0,0) of the reference frame.
    * @param mb MultiBody used has model.
    * @param mbc Uses force, parentToSon, bodyPosW, parentToSon,
    * motionSubspace
    * and gravity.
    * @param jacMomentsAndForces vector of jacobian of external forces on each
    * body. It is assumed to be computed by the user. It is the jacobian of the
    * equivalent force transported at the zero of the robot. Note that those
    * matrix should be empty if zero to avoid unnecessary calculations.
    * Fills jointTorqueJacQ and jointTorqueJacF.
    */
  void computeTorqueJacobianJoint(
      const MultiBody& mb, MultiBodyConfig& mbc,
      const std::vector<Eigen::MatrixXd>& jacMomentsAndForces);

  /**
    * Default version of computeTorqeuJacobienJoint
    * The external forces are assumed constant w.r.t q
    */
  void computeTorqueJacobianJoint(const MultiBody& mb, MultiBodyConfig& mbc);

  // safe version for python binding

  /** safe version of @see inverseStatics.
    * @throw std::domain_error If mb don't match mbc.
    */
  void sInverseStatics(const MultiBody& mb, MultiBodyConfig& mbc);

  /**
    * @brief Get the internal forces.
    * @return vector of forces transmitted from body λ(i) to body i across
    * joint i.
    */
  const std::vector<sva::ForceVecd>& f() const { return f_; };

  const std::vector<Eigen::MatrixXd>& jointTorqueDiff() const
  {
    return jointTorqueDiff_;
  };

 private:
  /// @brief Internal forces.
  /// f_ is the vector of forces transmitted from body λ(i) to body i across
  /// joint i.
  std::vector<sva::ForceVecd> f_;
  std::vector<Eigen::MatrixXd> df_;
  std::vector<Eigen::MatrixXd> jointTorqueDiff_;
  std::vector<Jacobian> jacW_;
  Eigen::MatrixXd fullJac_;
  bool jacobianSizeHasBeenSet_;
};

}  // namespace rbd
