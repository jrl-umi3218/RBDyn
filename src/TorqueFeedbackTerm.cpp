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

#include "RBDyn/TorqueFeedbackTerm.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

#include <time.h>

namespace torque_control
{

/**
 *    TorqueFeedbackTerm
 */
  
TorqueFeedbackTerm::TorqueFeedbackTerm(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                                       const std::shared_ptr<rbd::ForwardDynamics> fd)
  : nrDof_(mbs[robotIndex].nrDof()),
    fd_(fd),
    P_(Eigen::VectorXd::Zero(nrDof_)),
    gammaD_(Eigen::VectorXd::Zero(nrDof_))
{
  elapsed_ = {{"computeFbTerm-Gain", 0},
              {"computeFbTerm-Gain-Coriolis", 0},
              {"computeFbTerm-GammaD", 0}};
}

void TorqueFeedbackTerm::computeGammaD()
{
  // Alternative method to do
  // gammaD_ = fd_->H().inverse() * P_;
    
  gammaD_ = P_;
  LLT_.compute(fd_->H());
  LLT_.matrixL().solveInPlace(gammaD_);
  LLT_.matrixL().transpose().solveInPlace(gammaD_);
}

ElapsedTimeMap & TorqueFeedbackTerm::getElapsedTimes()
{
  return elapsed_;
}
  
/**
 *    IntegralTerm
 */

IntegralTerm::IntegralTerm(const std::vector<rbd::MultiBody> & mbs, int robotIndex,
                           const std::shared_ptr<rbd::ForwardDynamics> fd,
                           IntegralTermType intglTermType, VelocityGainType velGainType,
                           double lambda, double phiSlow, double phiFast, double fastFilterWeight, 
                           double timeStep)
  : TorqueFeedbackTerm(mbs, robotIndex, fd),
    intglTermType_(intglTermType),
    velGainType_(velGainType),
    lambda_(lambda),
    coriolis_(mbs[robotIndex]),
    C_(Eigen::MatrixXd::Zero(nrDof_, nrDof_)),
    L_(Eigen::MatrixXd::Zero(nrDof_, nrDof_)),
    previousS_(Eigen::VectorXd::Zero(nrDof_)),
    fastFilteredS_(Eigen::VectorXd::Zero(nrDof_)),
    slowFilteredS_(Eigen::VectorXd::Zero(nrDof_)),
    phiSlow_(phiSlow),
    phiFast_(phiFast),
    fastFilterWeight_(fastFilterWeight),
    timeStep_(timeStep)    
{
}

void IntegralTerm::computeGain(const rbd::MultiBody & mb,
			       const rbd::MultiBodyConfig & mbc_real)
{
  Eigen::MatrixXd K;

  // std::cout << "Rafa, in IntegralTerm::computeTerm, fd_->H().rows() = " << fd_->H().rows() << std::endl;
  
  if (velGainType_ == MassMatrix)
  {
    K = lambda_ * fd_->H();
  }
  else if (velGainType_ == MassDiagonal)
  {
    K = lambda_ * fd_->H().diagonal().asDiagonal();
  }
  else
  {
    K = lambda_ * Eigen::MatrixXd::Identity(nrDof_, nrDof_);
  }

  clock_t time;

  if (intglTermType_ == PassivityBased)
  {
    time = clock();
    C_ = coriolis_.coriolis(mb, mbc_real);
    elapsed_.at("computeFbTerm-Gain-Coriolis") = (int) (clock() - time);
    L_ = (C_ + K);
  }
  else
  {
    L_ = K;
  }
}

void IntegralTerm::computeTerm(const rbd::MultiBody & mb,
                               const rbd::MultiBodyConfig & mbc_real,
                               const rbd::MultiBodyConfig & mbc_calc)
{
  clock_t time;
  
  if (intglTermType_ == Simple || intglTermType_ == PassivityBased)
  {
    time = clock();
    computeGain(mb, mbc_real);
    elapsed_.at("computeFbTerm-Gain") = (int) (clock() - time);
    
    Eigen::VectorXd alphaVec_ref = rbd::dofToVector(mb, mbc_calc.alpha);
    Eigen::VectorXd alphaVec_hat = rbd::dofToVector(mb, mbc_real.alpha);
  
    Eigen::VectorXd s = alphaVec_ref - alphaVec_hat;
    
    slowFilteredS_ = exp(-timeStep_*phiSlow_) * slowFilteredS_ + s- previousS_;
    fastFilteredS_ = exp(-timeStep_*phiFast_) * fastFilteredS_ + s- previousS_;

    previousS_ = s;

    Eigen::VectorXd filteredS = fastFilterWeight_*fastFilteredS_+(1-fastFilterWeight_)*slowFilteredS_;
    
    P_ = L_ * filteredS;

    time = clock();
    computeGammaD();
    elapsed_.at("computeFbTerm-GammaD") = (int) (clock() - time);
  }
}

void IntegralTerm::computeTerm(const rbd::MultiBody & mb,
                               const rbd::MultiBodyConfig & mbc_real,
                               const rbd::MultiBodyConfig & mbc_calc,
                               const Eigen::VectorXd & diff_torques)
{
  slowFilteredS_ = diff_torques;
  computeTerm(mb, mbc_real, mbc_calc);
}

/**
 *    IntegralTermAntiWindup
 */

IntegralTermAntiWindup::IntegralTermAntiWindup(const std::vector<rbd::MultiBody> & mbs, int robotIndex,
					       const std::shared_ptr<rbd::ForwardDynamics> fd,
					       IntegralTermType intglTermType,
					       VelocityGainType velGainType,
					       double lambda, double perc,
					       const Eigen::Vector3d & maxLinAcc,
					       const Eigen::Vector3d & maxAngAcc,
					       const Eigen::VectorXd & torqueL,
					       const Eigen::VectorXd & torqueU,
                 double phiSlow,
                 double phiFast, 
                 double fastFilterWeight, 
                 double timeStep )
  : IntegralTerm(mbs, robotIndex, fd, intglTermType, velGainType, 
                lambda, phiSlow, phiFast, fastFilterWeight, timeStep),
    perc_(perc), maxLinAcc_(maxLinAcc), maxAngAcc_(maxAngAcc),
    torqueL_(torqueL), torqueU_(torqueU)
{}

void IntegralTermAntiWindup::computeTerm(const rbd::MultiBody & mb,
					 const rbd::MultiBodyConfig & mbc_real,
					 const rbd::MultiBodyConfig & mbc_calc)
{
  if (intglTermType_ == Simple || intglTermType_ == PassivityBased)
  {
    computeGain(mb, mbc_real);
    
    Eigen::VectorXd alphaVec_ref = rbd::dofToVector(mb, mbc_calc.alpha);
    Eigen::VectorXd alphaVec_hat = rbd::dofToVector(mb, mbc_real.alpha);
  
    Eigen::VectorXd s = alphaVec_ref - alphaVec_hat;
    P_ = L_ * s;

    // std::cout << "Rafa, in IntegralTermAntiWindup::computeTerm, torqueU_ = " << torqueU_.transpose() << std::endl;

    Eigen::VectorXd torqueU_prime = torqueU_ * perc_;
    Eigen::VectorXd torqueL_prime = torqueL_ * perc_;
    
    for (size_t i = 0; i < mb.nrJoints(); i++)
      if (mb.joint(i).type() == rbd::Joint::Free)
      {
        int j = mb.jointPosInDof(j);
        Eigen::Vector6d acc;
	acc << maxAngAcc_, maxLinAcc_;
        torqueU_prime.segment<6>(j) = fd_->H().block<6, 6>(j, j).diagonal().asDiagonal() * acc;
        torqueL_prime.segment<6>(j) = -torqueU_prime.segment<6>(j);
        break;
      }
    
    // Eigen::VectorXd torqueU_prime = (abs(torqueU_.array()) < 1E-6).select( max_float_, torqueU_);
    // Eigen::VectorXd torqueL_prime = (abs(torqueL_.array()) < 1E-6).select(-max_float_, torqueL_);

    //std::cout << "Rafa, in IntegralTermAntiWindup::computeTerm, P_ = " << P_.transpose() << std::endl;
    //std::cout << "Rafa, in IntegralTermAntiWindup::computeTerm, torqueU_prime = " << torqueU_prime.transpose() << std::endl;
    
    double epsilonU = (abs(P_.array() / torqueU_prime.array())).maxCoeff();
    double epsilonL = (abs(P_.array() / torqueL_prime.array())).maxCoeff();
    double epsilon  = std::max(epsilonU, epsilonL);

    // std::cout << "Rafa, in IntegralTermAntiWindup::computeTerm, epsilonU = " << epsilonU << ", epsilonL = " << epsilonL << ", epsilon = " << epsilon << std::endl << std::endl;
    
    if (epsilon > 1) {
      P_ /= epsilon;
      //std::cout << "Rafa, in IntegralTermAntiWindup::computeTerm, the AntiWindup is activated with gain = " << 1/epsilon << std::endl;
    }
    
    computeGammaD();
  }
}
  
/**
  *    PassivityPIDTerm
  */

PassivityPIDTerm::PassivityPIDTerm(const std::vector<rbd::MultiBody> & mbs, int robotIndex,
                                   const std::shared_ptr<rbd::ForwardDynamics> fd, double timeStep,
                                   double beta, double lambda, double mu, double sigma, double cis)
  : TorqueFeedbackTerm(mbs, robotIndex, fd),
    dt_(timeStep),
    beta_(beta),
    lambda_(lambda),
    mu_(mu),
    sigma_(sigma),
    cis_(cis),
    EPrev_(Eigen::VectorXd::Zero(nrDof_)),
    coriolis_(mbs[robotIndex]),
    C_(Eigen::MatrixXd::Zero(nrDof_, nrDof_))
{
}
  
void PassivityPIDTerm::computeTerm(const rbd::MultiBody & mb,
                                   const rbd::MultiBodyConfig & mbc_real,
                                   const rbd::MultiBodyConfig & mbc_calc)
{
  const Eigen::MatrixXd & M = fd_->H();
  
  C_ = coriolis_.coriolis(mb, mbc_real);

  Eigen::MatrixXd Ka = beta_  * M;
  //Eigen::MatrixXd L  = sigma_ * M.diagonal().asDiagonal();
  Eigen::MatrixXd L  = sigma_ * M;
  
  Eigen::MatrixXd Kv = lambda_ * M + C_ + Ka;
  Eigen::MatrixXd Kp = mu_ * M + lambda_ * (C_ + Ka) + L;
  Eigen::MatrixXd Ki = mu_ * (C_ + Ka) + cis_ * lambda_ * L;

  Eigen::VectorXd alphaVec_ref = rbd::dofToVector(mb, mbc_calc.alpha);
  Eigen::VectorXd alphaVec_hat = rbd::dofToVector(mb, mbc_real.alpha);

  Eigen::VectorXd s = alphaVec_ref - alphaVec_hat;

  const std::vector<rbd::Joint>& joints = mb.joints();

  Eigen::VectorXd e = Eigen::VectorXd::Zero(nrDof_);

  int pos = 0;
  for (std::size_t i = 0; i < joints.size(); i++) {
    Eigen::VectorXd ei = errorParam(joints[i].type(), mbc_calc.q[i], mbc_real.q[i]);
    e.segment(pos, ei.size()) = ei;
    pos += ei.size();
  }

  Eigen::VectorXd E = EPrev_ + e * dt_;
  EPrev_ = E;

  P_ = Kv * s + Kp * e + Ki * E;
  
  computeGammaD();
}

Eigen::VectorXd PassivityPIDTerm::errorParam(rbd::Joint::Type type,
                                             const std::vector<double> & q_ref,
                                             const std::vector<double> & q_hat)
{
  Eigen::VectorXd e;
  
  switch (type) {

  case rbd::Joint::Rev:
  case rbd::Joint::Prism:

    e.resize(1);
    e[0] = (q_ref[0] - q_hat[0]);
    break;

  case rbd::Joint::Free:

    e.resize(6);
    
    Eigen::Quaterniond quat_ref(q_ref[0], q_ref[1], q_ref[2], q_ref[3]);
    Eigen::Quaterniond quat_hat(q_hat[0], q_hat[1], q_hat[2], q_hat[3]);

    Eigen::MatrixXd Re = quat_ref.normalized().toRotationMatrix() * quat_hat.normalized().toRotationMatrix().transpose();
    Eigen::MatrixXd Sw = Re.log();

    e[0] = Sw(2, 1);
    e[1] = Sw(0, 2);
    e[2] = Sw(1, 0);

    for (std::size_t i = 0; i < 3; i++)
      e[3 + i] = q_ref[4 + i] - q_hat[4 + i];

    break;
  }

  return e;
}

} // namespace torque_control
