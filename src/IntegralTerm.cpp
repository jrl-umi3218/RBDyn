// Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
//
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

#include "RBDyn/IntegralTerm.h"

#include <RBDyn/Coriolis.h>

namespace integral
{

IntegralTerm::IntegralTerm(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                           const std::shared_ptr<rbd::ForwardDynamics> fd,
                           IntegralTermType intTermType, VelocityGainType velGainType,
                           double lambda):
  
  nrDof_(mbs[robotIndex].nrDof()),
  fd_(fd),
  intTermType_(intTermType),
  velGainType_(velGainType),
  lambda_(lambda),
  P_(Eigen::VectorXd::Zero(nrDof_)),
  gamma_(Eigen::VectorXd::Zero(nrDof_))
{
}

void IntegralTerm::computeTerm(const rbd::MultiBody& mb,
                               const rbd::MultiBodyConfig& mbc_real,
                               const rbd::MultiBodyConfig& mbc_calc)
{
  if (intTermType_ == Simple || intTermType_ == PassivityBased)
  {
    Eigen::MatrixXd K;
    
    if (velGainType_ == MassMatrix)
      {
        K = lambda_ * fd_->H();
      }
    else
      {
        K = lambda_ * Eigen::MatrixXd::Identity(nrDof_, nrDof_);
      }

    Eigen::VectorXd alphaVec_ref = rbd::dofToVector(mb, mbc_calc.alpha);
    Eigen::VectorXd alphaVec_hat = rbd::dofToVector(mb, mbc_real.alpha);
  
    Eigen::VectorXd s = alphaVec_ref - alphaVec_hat;
  
    if (intTermType_ == PassivityBased)
      {
        coriolis::Coriolis coriolis(mb);
        Eigen::MatrixXd C = coriolis.coriolis(mb, mbc_real);
        P_ = (C + K) * s;
      }
    else
      {
        P_ = K * s;
      }
  
    gamma_ = fd_->H().inverse() * P_;
  }
}

} // namespace integral
