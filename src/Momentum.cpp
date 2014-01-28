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

// associated header
#include "Momentum.h"

// includes
#include <iostream>
// RBDyn
#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "Jacobian.h"

namespace rbd
{


sva::ForceVecd computeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();
	Vector6d cm(Vector6d::Zero());

	sva::PTransformd X_com_0(Vector3d(-com));
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		// body inertia in body coordinate
		sva::ForceVecd hi = bodies[i].inertia()*mbc.bodyVelB[i];

		// transform in com coordinate
		cm += (mbc.bodyPosW[i]*X_com_0).transMul(hi).vector();
	}

	return sva::ForceVecd(cm);
}


/*
sva::ForceVecd computeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();
	Vector6d cm(Vector6d::Zero());

	sva::PTransformd X_com_0(Vector3d(-com));
	sva::MotionVecd comVel(Vector3d::Zero(), comDot);
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		// com velocity in body i frame
		sva::MotionVecd comVel_i(mbc.bodyPosW[i].rotation().transpose()*comVel);
		sva::MotionVecd iVel_com(mbc.bodyVelW[i]);
		sva::PTransformd X_com_i(mbc.bodyPosW[i]*X_com_0);
		sva::PTransformd X_i_com(X_com_i.inv());

		Matrix6d X_com_i_dot =
			sva::vector6ToCrossDualMatrix((mbc.bodyVelB[i] - comVel_i).vector())*X_com_i.inv().dualMatrix();

		Matrix6d X_i_com_dot =
			sva::vector6ToCrossDualMatrix((iVel_com - comVel).vector())*X_i_com.dualMatrix();
		sva::MotionVecd vec =iVel_com - comVel;
		vec = comVel_i - mbc.bodyVelW[i];
		//vec = X_com_i*comVel - mbc.bodyVelB[i];

		// transform in com coordinate
		cm += (vec.crossDual(X_i_com.dualMul(bodies[i].inertia()*mbc.bodyVelB[i]))).vector() +
				(X_com_i.transMul(bodies[i].inertia()*mbc.bodyAccB[i])).vector();
		cm += X_i_com_dot*((bodies[i].inertia()*mbc.bodyVelB[i]).vector()) +
				(X_com_i.transMul(bodies[i].inertia()*mbc.bodyAccB[i])).vector();
		cm += (bodies[i].inertia()*mbc.bodyAccB[i]).vector();
	}

	return sva::ForceVecd(cm);
}
*/


sva::ForceVecd sComputeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	return computeCentroidalMomentum(mb, mbc, com);
}



CentroidalMomentumMatrix::CentroidalMomentumMatrix():
	cmMat_(),
	jacFull_(),
	jacVec_(),
	jacWork_(),
	bodiesWeight_()
{}


CentroidalMomentumMatrix::CentroidalMomentumMatrix(const MultiBody& mb):
	cmMat_(6, mb.nrDof()),
	jacFull_(6, mb.nrDof()),
	jacVec_(mb.nrBodies()),
	jacWork_(mb.nrBodies()),
	bodiesWeight_(mb.nrBodies(), 1.)
{
	init(mb);
}


CentroidalMomentumMatrix::CentroidalMomentumMatrix(const MultiBody& mb,
	std::vector<double> weight):
	cmMat_(6*mb.nrBodies(), mb.nrDof()),
	jacFull_(6, mb.nrDof()),
	jacVec_(mb.nrBodies()),
	jacWork_(mb.nrBodies()),
	bodiesWeight_(std::move(weight))
{
	init(mb);

  if(int(bodiesWeight_.size()) != mb.nrBodies())
  {
    std::stringstream ss;
    ss << "weight vector must be of size " << mb.nrBodies() << " not " <<
          bodiesWeight_.size() << std::endl;
    throw std::domain_error(ss.str());
  }
}


const Eigen::MatrixXd& CentroidalMomentumMatrix::matrix(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	using namespace Eigen;
	const std::vector<Body>& bodies = mb.bodies();
	cmMat_.setZero();

	sva::PTransformd X_com_0(Vector3d(-com));
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacWork_[i] = bodies[i].inertia().matrix()*jacVec_[i].bodyJacobian(mb, mbc);
		jacWork_[i] = (mbc.bodyPosW[i]*X_com_0).inv().dualMatrix()*jacWork_[i];

		jacVec_[i].fullJacobian(mb, jacWork_[i], jacFull_);

		cmMat_.block(0, 0, 6, mb.nrDof()) += jacFull_;
	}

	return cmMat_;
}


const Eigen::MatrixXd& CentroidalMomentumMatrix::matrixDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	using namespace Eigen;
	const std::vector<Body>& bodies = mb.bodies();
	cmMat_.setZero();

	sva::PTransformd X_com_0(Vector3d(-com));
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacWork_[i] = bodies[i].inertia().matrix()*jacVec_[i].bodyJacobian(mb, mbc);
		jacWork_[i] = (mbc.bodyPosW[i]*X_com_0).inv().dualMatrix()*jacWork_[i];

		jacVec_[i].fullJacobian(mb, jacWork_[i], jacFull_);

		cmMat_.block(0, 0, 6, mb.nrDof()) += jacFull_;
	}

	return cmMat_;
}


void CentroidalMomentumMatrix::init(const rbd::MultiBody& mb)
{
	using namespace Eigen;
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacVec_[i] = Jacobian(mb, mb.body(i).id());
		jacWork_[i].resize(6, jacVec_[i].dof());
	}
}



} // rbd
