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

// associated header
#include "RBDyn/Momentum.h"

// includes
// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/Jacobian.h"

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

		// momentum at CoM for link i : {}^iX_{com}^T {}^iI_i {}^iV_i
		cm += (mbc.bodyPosW[i]*X_com_0).transMul(hi).vector();
	}

	return sva::ForceVecd(cm);
}


sva::ForceVecd computeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();
	Vector6d cm(Vector6d::Zero());

	sva::PTransformd X_com_0(Vector3d(-com));
	sva::MotionVecd com_Vel(Vector3d::Zero(), comDot);

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		sva::MotionVecd body_i_Vel(mbc.bodyVelB[i]);
		sva::PTransformd X_com_i(mbc.bodyPosW[i]*X_com_0);
		sva::PTransformd X_i_com(X_com_i.inv());

		sva::ForceVecd body_i_Momentum(bodies[i].inertia()*body_i_Vel);

		// momentum at CoM for link i : {}^iX_{com}^T {}^iI_i {}^iV_i
		// derivative :
		// \frac {d{}^iX_{com}^T}{dt} {}^iI_i {}^iV_i +
		//   {}^iX_{com}^T {}^iI_i \frac {d{}^iV_i}{dt}
		// {d{}^iX_{com}^T}{dt} =
		//   {}^{com}X_i^* {}^iV_i\times^* - {}^{com}V_{com}\times^* {}^{com}X_i^*
		// \frac {d{}^iV_i}{dt} =
		//   {}^iA_i
		// See Rigid Body Dynamics Algoritms - Roy Featherstone - P28 eq 2.45

		sva::ForceVecd X_i_com_d_dual_hi =
				X_i_com.dualMul(body_i_Vel.crossDual(body_i_Momentum)) -
				com_Vel.crossDual(X_i_com.dualMul(body_i_Momentum));

		// transform in com coordinate
		cm += X_i_com_d_dual_hi.vector() +
				(X_com_i.transMul(bodies[i].inertia()*mbc.bodyAccB[i])).vector();
	}

	return sva::ForceVecd(cm);
}


sva::ForceVecd sComputeCentroidalMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	return computeCentroidalMomentum(mb, mbc, com);
}


sva::ForceVecd sComputeCentroidalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchBodyAcc(mb, mbc);
	return computeCentroidalMomentumDot(mb, mbc, com, comDot);
}



Eigen::Matrix6d jacProjector(const sva::PTransformd& X_i_com,
	const sva::RBInertiad& I_i)
{
	return Eigen::Matrix6d(X_i_com.dualMatrix()*I_i.matrix());
}


Eigen::Matrix6d jacProjectorDot(const sva::PTransformd& X_i_com,
	const sva::RBInertiad& I_i, const sva::MotionVecd& V_i,
	const sva::MotionVecd& V_com)
{
	Eigen::Matrix6d X_i_com_d =
			X_i_com.dualMatrix()*sva::vector6ToCrossDualMatrix(V_i.vector()) -
			sva::vector6ToCrossDualMatrix(V_com.vector())*X_i_com.dualMatrix();
	return Eigen::Matrix6d(X_i_com_d*I_i.matrix());
}


CentroidalMomentumMatrix::CentroidalMomentumMatrix():
	cmMat_(),
	cmMatDot_(),
	jacVec_(),
	blocksVec_(),
	jacWork_(),
	bodiesWeight_()
{}


CentroidalMomentumMatrix::CentroidalMomentumMatrix(const MultiBody& mb):
	cmMat_(6, mb.nrDof()),
	cmMatDot_(6, mb.nrDof()),
	jacVec_(mb.nrBodies()),
	blocksVec_(mb.nrBodies()),
	jacWork_(mb.nrBodies()),
	bodiesWeight_(mb.nrBodies(), 1.),
	normalAcc_(mb.nrBodies())
{
	init(mb);
}


CentroidalMomentumMatrix::CentroidalMomentumMatrix(const MultiBody& mb,
	std::vector<double> weight):
	cmMat_(6, mb.nrDof()),
	cmMatDot_(6, mb.nrDof()),
	jacVec_(mb.nrBodies()),
	blocksVec_(mb.nrBodies()),
	jacWork_(mb.nrBodies()),
	bodiesWeight_(std::move(weight)),
	normalAcc_(mb.nrBodies())
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


void CentroidalMomentumMatrix::computeMatrix(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com)
{
	using namespace Eigen;
	const std::vector<Body>& bodies = mb.bodies();
	cmMat_.setZero();

	sva::PTransformd X_0_com(com);
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].bodyJacobian(mb, mbc);
		sva::PTransformd X_i_com(X_0_com*(mbc.bodyPosW[i].inv()));
		Matrix6d proj = bodiesWeight_[i]*jacProjector(X_i_com, bodies[i].inertia());

		jacWork_[i] = proj*jac;
		jacVec_[i].addFullJacobian(blocksVec_[i], jacWork_[i], cmMat_);
	}
}


void CentroidalMomentumMatrix::computeMatrixDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	using namespace Eigen;
	const std::vector<Body>& bodies = mb.bodies();
	cmMatDot_.setZero();

	sva::PTransformd X_0_com(com);
	sva::MotionVecd com_Vel(Vector3d::Zero(), comDot);
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].bodyJacobian(mb, mbc);
		const MatrixXd& jacDot = jacVec_[i].bodyJacobianDot(mb, mbc);

		sva::PTransformd X_i_com(X_0_com*(mbc.bodyPosW[i].inv()));
		Matrix6d proj = bodiesWeight_[i]*jacProjector(X_i_com, bodies[i].inertia());
		Matrix6d projDot = bodiesWeight_[i]*jacProjectorDot(X_i_com, bodies[i].inertia(),
			mbc.bodyVelB[i], com_Vel);

		jacWork_[i] = proj*jacDot + projDot*jac;
		jacVec_[i].addFullJacobian(blocksVec_[i], jacWork_[i], cmMatDot_);
	}
}


void CentroidalMomentumMatrix::computeMatrixAndMatrixDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	using namespace Eigen;
	const std::vector<Body>& bodies = mb.bodies();
	cmMat_.setZero();
	cmMatDot_.setZero();

	sva::PTransformd X_0_com(com);
	sva::MotionVecd com_Vel(Vector3d::Zero(), comDot);
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].bodyJacobian(mb, mbc);
		const MatrixXd& jacDot = jacVec_[i].bodyJacobianDot(mb, mbc);

		sva::PTransformd X_i_com(X_0_com*(mbc.bodyPosW[i].inv()));
		Matrix6d proj = bodiesWeight_[i]*jacProjector(X_i_com, bodies[i].inertia());
		Matrix6d projDot = bodiesWeight_[i]*jacProjectorDot(X_i_com, bodies[i].inertia(),
			mbc.bodyVelB[i], com_Vel);

		jacWork_[i] = proj*jac;
		jacVec_[i].addFullJacobian(blocksVec_[i], jacWork_[i], cmMat_);

		jacWork_[i] = proj*jacDot + projDot*jac;
		jacVec_[i].addFullJacobian(blocksVec_[i], jacWork_[i], cmMatDot_);
	}
}


const Eigen::MatrixXd& CentroidalMomentumMatrix::matrix() const
{
	return cmMat_;
}


const Eigen::MatrixXd& CentroidalMomentumMatrix::matrixDot() const
{
	return cmMatDot_;
}


sva::ForceVecd CentroidalMomentumMatrix::momentum(
	const MultiBody& mb, const MultiBodyConfig& mbc,
	const Eigen::Vector3d& com) const
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();
	Vector6d cm(Vector6d::Zero());

	sva::PTransformd X_com_0(Vector3d(-com));
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		// body inertia in body coordinate
		sva::ForceVecd hi = bodies[i].inertia()*mbc.bodyVelB[i];

		// momentum at CoM for link i : {}^iX_{com}^T {}^iI_i {}^iV_i
		cm += ((mbc.bodyPosW[i]*X_com_0).transMul(hi).vector())*bodiesWeight_[i];
	}

	return sva::ForceVecd(cm);
}


sva::ForceVecd CentroidalMomentumMatrix::normalMomentumDot(
	const MultiBody& mb, const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	using namespace Eigen;

	const std::vector<int>& pred = mb.predecessors();
	const std::vector<int>& succ = mb.successors();

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		const sva::PTransformd& X_p_i = mbc.parentToSon[i];
		const sva::MotionVecd& vj_i = mbc.jointVelocity[i];
		const sva::MotionVecd& vb_i = mbc.bodyVelB[i];

		if(pred[i] != -1)
			normalAcc_[succ[i]] = X_p_i*normalAcc_[pred[i]] + vb_i.cross(vj_i);
		else
			normalAcc_[succ[i]] = vb_i.cross(vj_i);
	}

	return normalMomentumDot(mb, mbc, com, comDot, normalAcc_);
}


sva::ForceVecd CentroidalMomentumMatrix::normalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot,
	const std::vector<sva::MotionVecd>& normalAccB) const
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();
	Vector6d cm(Vector6d::Zero());

	sva::PTransformd X_com_0(Vector3d(-com));
	sva::MotionVecd com_Vel(Vector3d::Zero(), comDot);

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		sva::MotionVecd body_i_Vel(mbc.bodyVelB[i]);
		sva::PTransformd X_com_i(mbc.bodyPosW[i]*X_com_0);
		sva::PTransformd X_i_com(X_com_i.inv());

		sva::ForceVecd body_i_Momentum(bodies[i].inertia()*body_i_Vel);

		// momentum at CoM for link i : {}^iX_{com}^T {}^iI_i {}^iV_i
		// derivative :
		// \frac {d{}^iX_{com}^T}{dt} {}^iI_i {}^iV_i +
		//   {}^iX_{com}^T {}^iI_i \frac {d{}^iV_i}{dt}
		// {d{}^iX_{com}^T}{dt} =
		//   {}^{com}X_i^* {}^iV_i\times^* - {}^{com}V_{com}\times^* {}^{com}X_i^*
		// \frac {d{}^iV_i}{dt} =
		//   {}^iA_i
		// See Rigid Body Dynamics Algoritms - Roy Featherstone - P28 eq 2.45

		sva::ForceVecd X_i_com_d_dual_hi =
				X_i_com.dualMul(body_i_Vel.crossDual(body_i_Momentum)) -
				com_Vel.crossDual(X_i_com.dualMul(body_i_Momentum));

		// transform in com coordinate
		cm += (X_i_com_d_dual_hi.vector() +
					(X_com_i.transMul(bodies[i].inertia()*normalAccB[i])).vector())*\
				bodiesWeight_[i];
	}

	return sva::ForceVecd(cm);
}


void CentroidalMomentumMatrix::sComputeMatrix(const MultiBody& mb, const MultiBodyConfig& mbc,
	const Eigen::Vector3d& com)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);
	computeMatrix(mb, mbc, com);
}


void CentroidalMomentumMatrix::sComputeMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
	const Eigen::Vector3d& com, const Eigen::Vector3d& comDot)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);
	computeMatrixDot(mb, mbc, com, comDot);
}


void CentroidalMomentumMatrix::sComputeMatrixAndMatrixDot(const MultiBody& mb, const MultiBodyConfig& mbc,
	const Eigen::Vector3d& com, const Eigen::Vector3d& comDot)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);
	computeMatrixAndMatrixDot(mb, mbc, com, comDot);
}


sva::ForceVecd CentroidalMomentumMatrix::sMomentum(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com) const
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);

	return momentum(mb, mbc, com);
}


sva::ForceVecd CentroidalMomentumMatrix::sNormalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return normalMomentumDot(mb, mbc, com, comDot);
}


sva::ForceVecd CentroidalMomentumMatrix::sNormalMomentumDot(const MultiBody& mb,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& com,
	const Eigen::Vector3d& comDot,
	const std::vector<sva::MotionVecd>& normalAccB) const
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchBodiesVector(mb, normalAccB, "normalAccB");

	return normalMomentumDot(mb, mbc, com, comDot, normalAccB);
}


void CentroidalMomentumMatrix::init(const rbd::MultiBody& mb)
{
	using namespace Eigen;
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacVec_[i] = Jacobian(mb, mb.body(i).name());
		blocksVec_[i] = jacVec_[i].compactPath(mb);
		jacWork_[i].resize(6, jacVec_[i].dof());
	}
}



} // rbd
