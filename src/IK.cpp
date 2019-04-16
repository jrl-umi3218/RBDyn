// This file is part of RBDyn.
//
// Copyright (C) 2012 - 2017 CNRS-AIST JRL, CNRS-UM LIRMM
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
#include "RBDyn/IK.h"

// includes
// RBDyn
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

//SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// STL
#include <limits>
#include <chrono>

namespace rbd
{

namespace {

struct CwiseRoundOp {
	CwiseRoundOp(const double& inf, const double& sup) : m_inf(inf), m_sup(sup) {}
	double operator()(const double& x) const { return x>m_inf && x<m_sup ? 0 : x; }
	double m_inf, m_sup;
};

} // anonymous

InverseKinematics::InverseKinematics(const MultiBody& mb, ik::IKParams ikParams):
	ikParams_(ikParams),
	flag_(ik::Flag::NoInit),
	nrConstraint_(0),
	svd_()
{
	ikParams_ = filterParams(ikParams);
}

void InverseKinematics::addConstraint(const MultiBody& mb, const std::string& bodyName, const sva::PTransformd& endEffector, 
	const sva::PTransformd& target, ConstraintType type)
{
	int pos = 0;
	if (constraints_.size() != 0)
	{
		pos = constraints_.back().posInFulljac + constraints_.back().nrConstr;
	} 

	int nrConstr = (type == ConstraintType::Full ? 6 : 3);
	nrConstraint_ += nrConstr;
	constraints_.push_back({mb.bodyIndexByName(bodyName), pos, nrConstr, type, endEffector, target, Jacobian(mb, bodyName)});
}

void InverseKinematics::addConstraint(const MultiBody& mb, const std::string& bodyName, const sva::PTransformd& endEffector, const Eigen::Vector3d& target)
{
	addConstraint(mb, bodyName, endEffector, target, ConstraintType::Position);
}

void InverseKinematics::addConstraint(const MultiBody& mb, const std::string& bodyName, const sva::PTransformd& endEffector, const Eigen::Matrix3d& target)
{
	addConstraint(mb, bodyName, endEffector, target, ConstraintType::Orientation);
}

bool InverseKinematics::inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc)
{
	using namespace std::chrono;

	high_resolution_clock::time_point t1, t2;
	t1 = high_resolution_clock::now();

	flag_ = ik::Flag::NoInit;
	A_.resize(nrConstraint_, mb.nrDof());
	A_.setZero();
	b_.resize(nrConstraint_);
	solveIter_ = 0;
	int dof = 0;
	rbd::forwardKinematics(mb, mbc);
	Eigen::MatrixXd jacMat;
	Eigen::Vector6d v = Eigen::Vector6d::Ones();
	Eigen::Vector3d rotErr;
	Eigen::VectorXd res = Eigen::VectorXd::Zero(nrConstraint_);
	while(solveIter_ < ikParams_.maxIteration)
	{
		computeAb(mb, mbc);
		//non-strict zeros in jacobian can be a problem...
		A_ = A_.unaryExpr(CwiseRoundOp(-ikParams_.roundThreshold, ikParams_.roundThreshold));
		svd_.compute(A_, Eigen::ComputeThinU | Eigen::ComputeThinV);

		if (b_.norm() < ikParams_.minCostValue)
		{
			flag_ = ik::Flag::Optimum;
			break;
		}

		res = svd_.solve(b_);

		dof = 0;
		for (int jInd = 0; jInd < mb.nrJoints(); ++jInd)
		{
			std::vector<double>& qi = mbc.q[jInd];
			for(auto &qv : qi)
			{
				qv += ikParams_.alpha * res(dof++);
			}
		}

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		t2 = high_resolution_clock::now();
		if (duration_cast<duration<double>>(t2 - t1).count() > ikParams_.maxTime)
		{
			flag_ = ik::Flag::Timed;
			break;
		}

		solveIter_++;
	}

	if (flag_ == ik::Flag::NoInit)
	{
		flag_ = ik::Flag::MaxIter;
	}

	solveTime_ = duration_cast<duration<double>>(t2 - t1).count();
	return flag_ == ik::Flag::Optimum ? true : false;
}

void InverseKinematics::computeAb(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	int jacPos, dof, shortJacPos;
	sva::PTransformd X_0_p;
	Eigen::VectorXd err;
	for (auto& constraint : constraints_)
	{
		// Compute A
		X_0_p = constraint.endEffector * mbc.bodyPosW[constraint.bodyIndex];
		const Eigen::MatrixXd& shortJac = constraint.jac.jacobian(mb, mbc, X_0_p);
		jacPos = 0;
		shortJacPos = (constraint.type == ConstraintType::Position ? 3 : 0);
		for (int jInd : constraint.jac.jointsPath())
		{
			dof = mb.joint(jInd).dof();
			A_.block(constraint.posInFulljac, mb.jointPosInDof(jInd), constraint.nrConstr, dof) = \
				shortJac.block(shortJacPos, jacPos, constraint.nrConstr, dof);
			jacPos += dof;
		}

		// Compute b
		switch (constraint.type)
		{
			case ConstraintType::Full:
				b_.segment<3>(constraint.posInFulljac) = sva::rotationError(X_0_p.rotation(), constraint.target.rotation());
				b_.segment<3>(constraint.posInFulljac + 3) = constraint.target.translation() - X_0_p.translation();
				break;
			case ConstraintType::Orientation:
				b_.segment<3>(constraint.posInFulljac) = sva::rotationError(X_0_p.rotation(), constraint.target.rotation());
				break;
			case ConstraintType::Position:
				b_.segment<3>(constraint.posInFulljac) = constraint.target.translation() - X_0_p.translation();
				break;
		}
	}
}

ik::IKParams InverseKinematics::filterParams(const ik::IKParams& ikParams) const
{
	ik::IKParams ikp(ikParams);
	if (ikp.maxIteration <= 0)
	{
		ikp.maxIteration = std::numeric_limits<int>::max();
	}

	if (ikp.maxTime <= 0.)
	{
		ikp.maxTime = std::numeric_limits<double>::max();
	}

	return ikp;
}

bool InverseKinematics::sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc)
{
	checkMatchQ(mb, mbc);
	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return inverseKinematics(mb, mbc);
}

void InverseKinematics::sIKParams(const ik::IKParams& ikParams)
{
	if (ikParams.alpha < ik::ALMOST_ZERO)
	{
		std::ostringstream str;
		str << "Alpha value must be superior to eps where eps is "
				<< ik::ALMOST_ZERO << ". Given value is " << ikParams.alpha;
		throw std::domain_error(str.str());
	}

	ikParams_ = filterParams(ikParams);
}

} // namespace rbd
