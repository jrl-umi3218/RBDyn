#include "RBDyn/Coriolis.h"

namespace rbd
{
Eigen::MatrixXd expand(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::MatrixXd& jacMat)
{
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(mb.nrDof(), mb.nrDof());
	expandAdd(jac, mb, jacMat, res);
	return res;
}

void expandAdd(const rbd::Jacobian& jac,
			const rbd::MultiBody& mb,
			const Eigen::MatrixXd& jacMat,
			Eigen::MatrixXd& res)
{
	int rowJac = 0;
	int colJac = 0;
	for(int i : jac.jointsPath())
	{
		colJac = 0;
		for(int j : jac.jointsPath())
		{
			res.block(mb.jointPosInDof(i), mb.jointPosInDof(j), mb.joint(i).dof(), mb.joint(j).dof())
				+= jacMat.block(rowJac, colJac, mb.joint(i).dof(), mb.joint(j).dof());
			colJac += mb.joint(j).dof();
		}
		rowJac += mb.joint(i).dof();
	}
}

std::vector<std::array<int, 3>> compactPath(const rbd::Jacobian& jac,
						const rbd::MultiBody& mb)
{
	std::vector<std::array<int, 3>> res;

	int start_block = mb.jointPosInDof(jac.jointsPath()[0]);
	int len_block = mb.joint(jac.jointsPath()[0]).dof();

	int start;
	int startJac = 0;

	for(int i : jac.jointsPath())
	{
		if(i == jac.jointsPath()[0])
		{
			continue;
		}

		start = mb.jointPosInDof(i);

		if(start != start_block + len_block)
		{
			res.push_back({{start_block, startJac, len_block}});
			start_block = start;
			startJac += len_block;
			len_block = 0;
		}
		len_block += mb.joint(i).dof();
	}
	res.push_back({{start_block, startJac, len_block}});
	return res;
}

void compactExpandAdd(const std::vector<std::array<int, 3>>& compactPath,
			const Eigen::MatrixXd& jacMat,
			Eigen::MatrixXd& res)
{
	for(const auto& b1 : compactPath)
	{
		for(const auto& b2 : compactPath)
		{
			res.block(b1[0], b2[0], b1[2], b2[2])
				+= jacMat.block(b1[1], b2[1], b1[2], b2[2]);
		}
	}
}

Coriolis::Coriolis(const rbd::MultiBody& mb)
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
		jacMats_.push_back(Eigen::MatrixXd(6, jacs_[i].dof()));
		jacDotMats_.push_back(Eigen::MatrixXd(6, jacs_[i].dof()));
		compactPaths_.push_back(compactPath(jacs_[i], mb));
	}
}

Eigen::MatrixXd Coriolis::coriolis(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc)
{
	Eigen::MatrixXd coriolis = Eigen::MatrixXd::Zero(mb.nrDof(), mb.nrDof());

	Eigen::Matrix3d rot;
	Eigen::Matrix3d rDot;

	Eigen::Matrix3d inertia;

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		jacMats_[i] = jacs_[i].jacobian(mb, mbc);

		jacDotMats_[i] = jacs_[i].jacobianDot(mb, mbc);

		rot = mbc.bodyPosW[i].rotation().transpose();
		rDot.noalias() = sva::vector3ToCrossMatrix(mbc.bodyVelW[i].angular())*rot;

		auto jvi = jacMats_[i].bottomRows<3>();
		auto jDvi = jacDotMats_[i].bottomRows<3>();

		auto jwi = jacMats_[i].topRows<3>();
		auto jDwi = jacDotMats_[i].topRows<3>();

		double mass = mb.body(i).inertia().mass();
		inertia = mb.body(i).inertia().inertia()
			- sva::vector3ToCrossMatrix<double>(mass*jacs_[i].point())*sva::vector3ToCrossMatrix(jacs_[i].point()).transpose();

		Eigen::Matrix3d ir = inertia*rot.transpose();

		/* C = \sum m_i J_{v_i}^T \dot{J}_{v_i}
		 *        + J_{w_i}^T R_i I_i R_i^T \dot{J}_{w_i}
		 *        + J_{w_i}^T \dot{R}_i I_i R_i^T J_{w_i} */

		Eigen::MatrixXd res = mass*jvi.transpose()*jDvi
				+ jwi.transpose()*((rot*ir)*jDwi
				+  (rDot*ir)*jwi);

		compactExpandAdd(compactPaths_[i], res, coriolis);
	}

	return coriolis;
}

} //ns rbd
