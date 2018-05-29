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
Eigen::MatrixXd expand(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::Ref<const Eigen::MatrixXd>& jacMat)
{
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(mb.nrDof(), mb.nrDof());
	expandAdd(jac, mb, jacMat, res);
	return res;
}

void expandAdd(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::Ref<const Eigen::MatrixXd>& jacMat,
			Eigen::MatrixXd& res)
{
	assert(res.cols() == mb.nrDof() && res.rows() == mb.nrDof());
	int rowJac = 0;
	int colJac = 0;
	for(int i : jac.jointsPath())
	{
		colJac = 0;
		for(int j : jac.jointsPath())
		{
			res.block(mb.jointPosInDof(i), mb.jointPosInDof(j), mb.joint(i).dof(), mb.joint(j).dof()).noalias()
				+= jacMat.block(rowJac, colJac, mb.joint(i).dof(), mb.joint(j).dof());
			colJac += mb.joint(j).dof();
		}
		rowJac += mb.joint(i).dof();
	}
}

Blocks compactPath(const rbd::Jacobian& jac,
						const rbd::MultiBody& mb)
{
	Blocks res;

	int start_block = mb.jointPosInDof(jac.jointsPath()[0]);
	int len_block = mb.joint(jac.jointsPath()[0]).dof();

	int startJac = 0;

	const auto & jPath = jac.jointsPath();
	for (std::size_t j = 1;  j < jPath.size(); ++j)
	{
		int i = jPath[j];
		int start = mb.jointPosInDof(i);

		if(start != start_block + len_block)
		{
			res.emplace_back(start_block, startJac, len_block);
			start_block = start;
			startJac += len_block;
			len_block = 0;
		}
		len_block += mb.joint(i).dof();
	}
	res.emplace_back(start_block, startJac, len_block);
	return res;
}

void expandAdd(const Blocks& compactPath,
			const Eigen::Ref<const Eigen::MatrixXd>& jacMat,
			Eigen::MatrixXd& res)
{
	for(const auto& b1 : compactPath)
	{
		for(const auto& b2 : compactPath)
		{
			res.block(b1.startDof, b2.startDof, b1.length, b2.length).noalias()
				+= jacMat.block(b1.startJac, b2.startJac, b1.length, b2.length);
		}
	}
}

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
		compactPaths_.push_back(compactPath(jacs_[i], mb));
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

		expandAdd(compactPaths_[i], res_.topLeftCorner(jacs_[i].dof(), jacs_[i].dof()), coriolis_);
	}

	return coriolis_;
}

} //ns rbd
