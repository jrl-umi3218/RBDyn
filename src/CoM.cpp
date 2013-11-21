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
#include "CoM.h"

// RBDyn
#include "Jacobian.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"

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
		Vector3d comT = bodies[i].inertia().momentum()/mass;

		totalMass += mass;
		com += (sva::PTransformd(comT)*mbc.bodyPosW[i]).translation()*mass;
	}

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
		Vector3d comT = bodies[i].inertia().momentum()/mass;

		totalMass += mass;

		// Velocity at CoM : com_T_b·V_b
		// Velocity at CoM world frame : 0_R_b·com_T_b·V_b
		sva::PTransformd X_0_i(mbc.bodyPosW[i].rotation().transpose(), comT);
		comV += (X_0_i*mbc.bodyVelB[i]).linear()*mass;
	}

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
		Vector3d comT = bodies[i].inertia().momentum()/mass;

		totalMass += mass;

		// Acceleration at CoM : com_T_b·A_b
		// Acceleration at CoM world frame :
		//    0_R_b·com_T_b·A_b + 0_R_b_d·com_T_b·V_b
		// O_R_b_d : (Angvel_W)_b x 0_R_b
		sva::PTransformd X_0_i(mbc.bodyPosW[i].rotation().transpose(), comT);
		sva::MotionVecd angvel_W(mbc.bodyVelW[i].angular(), Eigen::Vector3d::Zero());
		comA += (X_0_i*mbc.bodyAccB[i]).linear()*mass;
		comA += (angvel_W.cross(X_0_i*mbc.bodyVelB[i])).linear()*mass;
	}

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



CoMJacobianDummy::CoMJacobianDummy()
{}


CoMJacobianDummy::CoMJacobianDummy(const MultiBody& mb):
  jac_(6, mb.nrDof()),
  jacDot_(6, mb.nrDof()),
  jacFull_(6, mb.nrDof()),
  jacVec_(mb.nrBodies()),
  totalMass_(0.)
{
  using namespace Eigen;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		Vector3d comT = mb.body(i).inertia().momentum()/
			mb.body(i).inertia().mass();
		jacVec_[i] = Jacobian(mb, mb.body(i).id(), comT);
		totalMass_ += mb.body(i).inertia().mass();
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
		jacVec_[i].fullJacobian(mb, jac, jacFull_);
		jac_ += jacFull_*bodies[i].inertia().mass();
	}

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
		jacVec_[i].fullJacobian(mb, jac, jacFull_);
		jacDot_ += jacFull_*bodies[i].inertia().mass();
	}

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

} // namespace rbd

