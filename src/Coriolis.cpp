// Copyright 2012-2018 CNRS-UM LIRMM, CNRS-AIST JRL
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

#include "RBDyn/Coriolis.h"

namespace rbd
{

Coriolis::Coriolis(const rbd::MultiBody& mb)
  : coriolis_(mb.nrDof(), mb.nrDof()),
		res_(0, 0)
{
	Eigen::Vector3d com;
	double mass;
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		mass = mb.body(i).inertia().mass();
		if(mass > 0)
		{
			com = mb.body(i).inertia().momentum()/mass;
		}
		else
		{
			com.setZero();
		}
		jacs_.push_back(rbd::Jacobian(mb, mb.body(i).name(), com));
		compactPaths_.push_back(jacs_.back().compactPath(mb));
		if(jacs_.back().dof() > res_.rows())
		{
			res_.resize(jacs_.back().dof(), jacs_.back().dof());
		}
	}
}

const Eigen::MatrixXd& Coriolis::coriolis(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc)
{
	Eigen::Matrix3d rot;
	Eigen::Matrix3d rDot;

	Eigen::Matrix3d inertia;

	coriolis_.setZero();

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		const auto & jac = jacs_[i].jacobian(mb, mbc);
		const auto & jacDot = jacs_[i].jacobianDot(mb, mbc);

		rot = mbc.bodyPosW[i].rotation().transpose();
		rDot.noalias() = sva::vector3ToCrossMatrix(mbc.bodyVelW[i].angular())*rot;

		auto jvi = jac.bottomRows<3>();
		auto jDvi = jacDot.bottomRows<3>();

		auto jwi = jac.topRows<3>();
		auto jDwi = jacDot.topRows<3>();

		double mass = mb.body(i).inertia().mass();
		inertia = mb.body(i).inertia().inertia()
			- sva::vector3ToCrossMatrix<double>(mass*jacs_[i].point())*sva::vector3ToCrossMatrix(jacs_[i].point()).transpose();

		Eigen::Matrix3d ir = inertia*rot.transpose();

		/* C = \sum m_i J_{v_i}^T \dot{J}_{v_i}
		 *        + J_{w_i}^T R_i I_i R_i^T \dot{J}_{w_i}
		 *        + J_{w_i}^T \dot{R}_i I_i R_i^T J_{w_i} */

		res_.topLeftCorner(jacs_[i].dof(), jacs_[i].dof()).noalias() =
			mass*jvi.transpose()*jDvi
			+ jwi.transpose()*(rot*ir*jDwi + rDot*ir*jwi);

		jacs_[i].expandAdd(compactPaths_[i], res_.topLeftCorner(jacs_[i].dof(), jacs_[i].dof()), coriolis_);
	}

	return coriolis_;
}

} //ns rbd
