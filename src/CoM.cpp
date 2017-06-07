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
#include "RBDyn/CoM.h"

// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"

namespace rbd
{

Eigen::Vector3d computeCoM(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	Vector3d com = Vector3d::Zero();
	double totalMass = 0.;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		double mass = bodies[i].inertia().mass();

		totalMass += mass;
		sva::PTransformd scaledBobyPosW(mbc.bodyPosW[i].rotation(), mass*mbc.bodyPosW[i].translation());
		com += (sva::PTransformd(bodies[i].inertia().momentum())*scaledBobyPosW).translation();
	}

	assert(totalMass > 0 && "Invalid multibody. Totalmass must be strictly positive");
	return com/totalMass;
}


Eigen::Vector3d computeCoMVelocity(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	Vector3d comV = Vector3d::Zero();
	double totalMass = 0.;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		double mass = bodies[i].inertia().mass();
		totalMass += mass;

		// Velocity at CoM : com_T_b·V_b
		// Velocity at CoM world frame : 0_R_b·com_T_b·V_b
		sva::PTransformd X_0_i(mbc.bodyPosW[i].rotation().transpose(), bodies[i].inertia().momentum());
		sva::MotionVecd scaledBodyVelB(mbc.bodyVelB[i].angular() , mass*mbc.bodyVelB[i].linear());
		comV += (X_0_i*scaledBodyVelB).linear();
	}

	assert(totalMass > 0 && "Invalid multibody. Totalmass must be strictly positive");
	return comV/totalMass;
}


Eigen::Vector3d computeCoMAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	Vector3d comA = Vector3d::Zero();
	double totalMass = 0.;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		double mass = bodies[i].inertia().mass();

		totalMass += mass;

		// Acceleration at CoM : com_T_b·A_b
		// Acceleration at CoM world frame :
		//    0_R_b·com_T_b·A_b + 0_R_b_d·com_T_b·V_b
		// O_R_b_d : (Angvel_W)_b x 0_R_b
		sva::PTransformd X_0_iscaled(mbc.bodyPosW[i].rotation().transpose(), bodies[i].inertia().momentum());
		sva::MotionVecd angvel_W(mbc.bodyVelW[i].angular(), Eigen::Vector3d::Zero());

		sva::MotionVecd scaledBodyAccB(mbc.bodyAccB[i].angular() , mass*mbc.bodyAccB[i].linear());
		sva::MotionVecd scaledBodyVelB(mbc.bodyVelB[i].angular() , mass*mbc.bodyVelB[i].linear());

		comA += (X_0_iscaled*scaledBodyAccB).linear();
		comA += (angvel_W.cross(X_0_iscaled*scaledBodyVelB)).linear();
	}

	assert(totalMass > 0 && "Invalid multibody. Totalmass must be strictly positive");
	return comA/totalMass;
}


Eigen::Vector3d sComputeCoM(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	return computeCoM(mb, mbc);
}


Eigen::Vector3d sComputeCoMVelocity(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	return computeCoMVelocity(mb, mbc);
}


Eigen::Vector3d sComputeCoMAcceleration(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchBodyAcc(mb, mbc);
	return computeCoMAcceleration(mb, mbc);
}


/**
	*														CoMJacobianDummy
	*/


CoMJacobianDummy::CoMJacobianDummy()
{}


CoMJacobianDummy::CoMJacobianDummy(const MultiBody& mb):
	jac_(3, mb.nrDof()),
	jacDot_(3, mb.nrDof()),
	jacFull_(3, mb.nrDof()),
	jacVec_(mb.nrBodies()),
	totalMass_(0.),
	bodiesWeight_(mb.nrBodies(), 1.)
{
	init(mb);
}


CoMJacobianDummy::CoMJacobianDummy(const MultiBody& mb, std::vector<double> weight):
  jac_(3, mb.nrDof()),
  jacDot_(3, mb.nrDof()),
  jacFull_(3, mb.nrDof()),
  jacVec_(mb.nrBodies()),
  totalMass_(0.),
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

CoMJacobianDummy::~CoMJacobianDummy()
{}


const Eigen::MatrixXd&
CoMJacobianDummy::jacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	jac_.setZero();

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].jacobian(mb, mbc);
		jacVec_[i].fullJacobian(mb, jac.block(3, 0, 3, jac.cols()), jacFull_);
		jac_.noalias() += jacFull_*(bodies[i].inertia().mass()*bodiesWeight_[i]);
	}

	assert(totalMass_ > 0 && "Invalid multibody. Totalmass must be strictly positive");
	jac_ /= totalMass_;

	return jac_;
}


const Eigen::MatrixXd&
CoMJacobianDummy::jacobianDot(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	using namespace Eigen;

	const std::vector<Body>& bodies = mb.bodies();

	jacDot_.setZero();

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const MatrixXd& jac = jacVec_[i].jacobianDot(mb, mbc);
		jacVec_[i].fullJacobian(mb, jac.block(3, 0, 3, jac.cols()), jacFull_);
		jacDot_.noalias() += jacFull_*(bodies[i].inertia().mass()*bodiesWeight_[i]);
	}

	assert(totalMass_ > 0 && "Invalid multibody. Totalmass must be strictly positive");
	jacDot_ /= totalMass_;

	return jacDot_;
}


const Eigen::MatrixXd&
CoMJacobianDummy::sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	return jacobian(mb, mbc);
}


const Eigen::MatrixXd&
CoMJacobianDummy::sJacobianDot(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	return jacobianDot(mb, mbc);
}


void CoMJacobianDummy::init(const rbd::MultiBody& mb)
{
	using namespace Eigen;
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		double bodyMass = mb.body(i).inertia().mass();
		Vector3d comT(0,0,0);
		if(bodyMass > 0)
			comT = mb.body(i).inertia().momentum()/bodyMass;

		jacVec_[i] = Jacobian(mb, mb.body(i).name(), comT);
		totalMass_ += mb.body(i).inertia().mass();
	}
}


/**
	*														CoMJacobian
	*/


CoMJacobian::CoMJacobian()
{}


CoMJacobian::CoMJacobian(const MultiBody& mb):
	jac_(3, mb.nrDof()),
	jacDot_(3, mb.nrDof()),
	bodiesCoeff_(mb.nrBodies()),
	bodiesCoM_(mb.nrBodies()),
	jointsSubBodies_(mb.nrJoints()),
	bodiesCoMWorld_(mb.nrBodies()),
	bodiesCoMVelB_(mb.nrBodies()),
	normalAcc_(mb.nrJoints()),
	weight_(mb.nrBodies(), 1.)
{
	init(mb);
}


CoMJacobian::CoMJacobian(const MultiBody& mb, std::vector<double> weight):
	jac_(3, mb.nrDof()),
	jacDot_(3, mb.nrDof()),
	bodiesCoeff_(mb.nrBodies()),
	bodiesCoM_(mb.nrBodies()),
	jointsSubBodies_(mb.nrJoints()),
	bodiesCoMWorld_(mb.nrBodies()),
	bodiesCoMVelB_(mb.nrBodies()),
	normalAcc_(mb.nrJoints()),
	weight_(std::move(weight))
{
	if(int(weight_.size()) != mb.nrBodies())
	{
		std::stringstream ss;
		ss << "weight vector must be of size " << mb.nrBodies() << " not " <<
					weight_.size() << std::endl;
		throw std::domain_error(ss.str());
	}

	init(mb);
}


void CoMJacobian::updateInertialParameters(const MultiBody& mb)
{
	double mass = 0.;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		mass += mb.body(i).inertia().mass();
	}

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		double bodyMass = mb.body(i).inertia().mass();
		bodiesCoeff_[i] = (bodyMass*weight_[i])/mass;
		if(bodyMass > 0)
			bodiesCoM_[i] = sva::PTransformd((mb.body(i).inertia().momentum()/bodyMass).eval());
		else
			bodiesCoM_[i] = sva::PTransformd::Identity();
	}
}


const std::vector<double>& CoMJacobian::weight() const
{
	return weight_;
}


void CoMJacobian::weight(const MultiBody& mb, std::vector<double> w)
{
	weight_ = std::move(w);
	updateInertialParameters(mb);
}


const Eigen::MatrixXd& CoMJacobian::jacobian(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	jac_.setZero();

	// we pre compute the CoM position of each bodie in world frame
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		// the transformation must be read {}^0E_p {}^pT_N {}^NX_0
		sva::PTransformd X_0_com_w = bodiesCoM_[i]*mbc.bodyPosW[i];
		bodiesCoMWorld_[i] = sva::PTransformd(X_0_com_w.translation());
	}

	int curJ = 0;
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		std::vector<int>& subBodies = jointsSubBodies_[i];
		sva::PTransformd X_i_0 = mbc.bodyPosW[i].inv();
		for(int b: subBodies)
		{
			sva::PTransformd X_i_com = bodiesCoMWorld_[b]*X_i_0;
			for(int dof = 0; dof < joints[i].dof(); ++dof)
			{
				jac_.col(curJ + dof).noalias() +=
					(X_i_com.linearMul(sva::MotionVecd(mbc.motionSubspace[i].col(dof))))*
						bodiesCoeff_[b];
			}
		}
		curJ += joints[i].dof();
	}

	return jac_;
}


const Eigen::MatrixXd& CoMJacobian::jacobianDot(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	jacDot_.setZero();

	// we pre compute the CoM position/velocity of each bodie
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		bodiesCoMWorld_[i] = bodiesCoM_[i]*mbc.bodyPosW[i];
		bodiesCoMVelB_[i] = bodiesCoM_[i]*mbc.bodyVelB[i];
	}

	int curJ = 0;
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		std::vector<int>& subBodies = jointsSubBodies_[i];
		sva::PTransformd X_i_0 = mbc.bodyPosW[i].inv();

		for(int b: subBodies)
		{
			sva::PTransformd X_i_com = bodiesCoMWorld_[b]*X_i_0;
			sva::PTransformd E_b_0(Eigen::Matrix3d(mbc.bodyPosW[b].rotation().transpose()));

			// angular velocity of rotation N to O
			sva::MotionVecd E_Vb(mbc.bodyVelW[b].angular(), Eigen::Vector3d::Zero());
			sva::MotionVecd X_Vcom_i_com = X_i_com*mbc.bodyVelB[i] - bodiesCoMVelB_[b];

			for(int dof = 0; dof < joints[i].dof(); ++dof)
			{
				sva::MotionVecd S_ij(mbc.motionSubspace[i].col(dof));

				// JD_i = (E_com_0_d*X_i_com*S_i + E_com_0*X_i_com_d*S_i)*(mass/totalMass)
				// E_com_0_d = (ANG_Vcom)_0 x E_com_0
				// X_i_com_d = (Vi - Vcom)_com x X_i_com
				jacDot_.col(curJ + dof).noalias() +=
					((E_Vb.cross(E_b_0*X_i_com*S_ij)).linear() +
					(E_b_0*X_Vcom_i_com.cross(X_i_com*S_ij)).linear())*bodiesCoeff_[b];
			}
		}
		curJ += joints[i].dof();
	}

	return jacDot_;
}


Eigen::Vector3d CoMJacobian::velocity(const MultiBody& mb,
	const MultiBodyConfig& mbc) const
{
	Eigen::Vector3d comV = Eigen::Vector3d::Zero();
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const Eigen::Vector3d& comT = bodiesCoM_[i].translation();

		// Velocity at CoM : com_T_b·V_b
		// Velocity at CoM world frame : 0_R_b·com_T_b·V_b
		sva::PTransformd X_0_i(mbc.bodyPosW[i].rotation().transpose(), comT);
		comV += (X_0_i*mbc.bodyVelB[i]).linear()*bodiesCoeff_[i];
	}

	return comV;
}


Eigen::Vector3d CoMJacobian::normalAcceleration(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
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

	return normalAcceleration(mb, mbc, normalAcc_);
}


Eigen::Vector3d CoMJacobian::normalAcceleration(const MultiBody& mb,
	const MultiBodyConfig& mbc, const std::vector<sva::MotionVecd>& normalAccB) const
{
	Eigen::Vector3d comA(Eigen::Vector3d::Zero());
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const Eigen::Vector3d& comT = bodiesCoM_[i].translation();

		// Normal Acceleration at CoM : com_T_b·A_b
		// Normal Acceleration at CoM world frame :
		//    0_R_b·com_T_b·A_b + 0_R_b_d·com_T_b·V_b
		// O_R_b_d : (Angvel_W)_b x 0_R_b
		sva::PTransformd X_0_i(mbc.bodyPosW[i].rotation().transpose(), comT);
		sva::MotionVecd angvel_W(mbc.bodyVelW[i].angular(), Eigen::Vector3d::Zero());
		comA += (X_0_i*normalAccB[i]).linear()*bodiesCoeff_[i];
		comA += (angvel_W.cross(X_0_i*mbc.bodyVelB[i])).linear()*bodiesCoeff_[i];
	}

	return comA;
}


void CoMJacobian::sUpdateInertialParameters(const MultiBody& mb)
{
	if(int(bodiesCoeff_.size()) != mb.nrBodies())
	{
		std::stringstream ss;
		ss << "mb should have " << bodiesCoeff_.size() << " bodies, not " <<
					mb.nrBodies() << std::endl;
		throw std::domain_error(ss.str());
	}

	updateInertialParameters(mb);
}


void CoMJacobian::sWeight(const MultiBody& mb, std::vector<double> w)
{
	if(int(bodiesCoeff_.size()) != mb.nrBodies())
	{
		std::stringstream ss;
		ss << "mb should have " << bodiesCoeff_.size() << " bodies, not " <<
					mb.nrBodies() << std::endl;
		throw std::domain_error(ss.str());
	}

	if(int(weight_.size()) != mb.nrBodies())
	{
		std::stringstream ss;
		ss << "weight vector must be of size " << mb.nrBodies() << " not " <<
					weight_.size() << std::endl;
		throw std::domain_error(ss.str());
	}

	weight(mb, w);
}


const Eigen::MatrixXd&
CoMJacobian::sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	return jacobian(mb, mbc);
}


const Eigen::MatrixXd&
CoMJacobian::sJacobianDot(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	return jacobianDot(mb, mbc);
}


Eigen::Vector3d CoMJacobian::sVelocity(const MultiBody& mb,
	const MultiBodyConfig& mbc) const
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);

	return velocity(mb, mbc);
}


Eigen::Vector3d CoMJacobian::sNormalAcceleration(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return normalAcceleration(mb, mbc);
}

Eigen::Vector3d CoMJacobian::sNormalAcceleration(const MultiBody& mb,
	const MultiBodyConfig& mbc, const std::vector<sva::MotionVecd>& normalAccB) const
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchBodiesVector(mb, normalAccB, "normalAccB");

	return normalAcceleration(mb, mbc, normalAccB);
}


// inefficient but the best we can do without mbg
void jointBodiesSuccessors(const MultiBody& mb, int joint, std::vector<int>& subBodies)
{
	int sonBody = mb.successor(joint);
	subBodies.push_back(sonBody);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		if(mb.predecessor(i) == sonBody)
		{
			jointBodiesSuccessors(mb, i, subBodies);
		}
	}
}


void CoMJacobian::init(const MultiBody& mb)
{
	updateInertialParameters(mb);

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		std::vector<int>& subBodies = jointsSubBodies_[i];
		jointBodiesSuccessors(mb, i, subBodies);
	}
}


} // namespace rbd

