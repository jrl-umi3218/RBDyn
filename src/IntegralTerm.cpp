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
#include <iostream>

namespace integral
{

IntegralTerm::IntegralTerm(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                           const std::shared_ptr<rbd::ForwardDynamics> fd,
                           IntegralTermType intglTermType, VelocityGainType velGainType,
                           double lambda):
  
  nrDof_(mbs[robotIndex].nrDof()),
  fd_(fd),
  intglTermType_(intglTermType),
  velGainType_(velGainType),
  lambda_(lambda),
  P_(Eigen::VectorXd::Zero(nrDof_)),
  gammaD_(Eigen::VectorXd::Zero(nrDof_))
{
}

void IntegralTerm::computeTerm(const rbd::MultiBody& mb,
                               const rbd::MultiBodyConfig& mbc_real,
                               const rbd::MultiBodyConfig& mbc_calc)
{
  if (intglTermType_ == Simple || intglTermType_ == PassivityBased)
  {
    Eigen::MatrixXd K;

    // std::cout << "Rafa, the size of fd_->H() is " << fd_->H().size() << std::endl;
    
    if (velGainType_ == MassMatrix)
    {
        K = lambda_ * fd_->H();
    }
    else if (velGainType_ == MassDiagonal)
    {
        K = lambda_ * fd_->H().diagonal().asDiagonal();

	std::cout << "Rafa, MassDiagonal = " << fd_->H().diagonal().transpose() << std::endl << std::endl;
    }
    else
    {
        K = lambda_ * Eigen::MatrixXd::Identity(nrDof_, nrDof_);
    }

    Eigen::VectorXd alphaVec_ref = rbd::dofToVector(mb, mbc_calc.alpha);
    Eigen::VectorXd alphaVec_hat = rbd::dofToVector(mb, mbc_real.alpha);
  
    Eigen::VectorXd s = alphaVec_ref - alphaVec_hat;

    std::cout << "Rafa, inside of IntegralTerm::computeTerm, alphaVec_ref = " << alphaVec_ref.transpose() << std::endl << std::endl;
    std::cout << "Rafa, inside of IntegralTerm::computeTerm, alphaVec_hat = " << alphaVec_hat.transpose() << std::endl << std::endl;
    
    std::cout << "Rafa, inside of IntegralTerm::computeTerm, s = " << s.transpose() << std::endl << std::endl;

    if (intglTermType_ == PassivityBased)
    {
        coriolis::Coriolis coriolis(mb);
        Eigen::MatrixXd C = coriolis.coriolis(mb, mbc_real);
        P_ = (C + K) * s;
    }
    else
    {
        P_ = K * s;
    }

    std::cout << "Rafa, inside of IntegralTerm::computeTerm, P_ = " << P_.transpose() << std::endl << std::endl;
    
    // Alternative method to do
    // gammaD_ = fd_->H().inverse() * P_;

    /*
    gammaD_ = P_;
    L_.compute(fd_->H());
    L_.matrixL().solveInPlace(gammaD_);
    L_.matrixL().transpose().solveInPlace(gammaD_);
    */
    
    // std::cout << "Rafa, gammaD_ = " << gammaD_.transpose() << std::endl;

    // std::cout << "Rafa, fd_->H() = " << std::endl << fd_->H() << std::endl;
  }
}

} // namespace integral
