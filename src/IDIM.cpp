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
#include "RBDyn/IDIM.h"

// includes
// RBDyn
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"


namespace rbd
{

Eigen::Matrix<double, 6, 10> IMPhi(const sva::MotionVecd& mv)
{
	using namespace Eigen;
	const Vector3d& w = mv.angular();
	const Vector3d& v = mv.linear();
	Matrix<double, 3, 6> iwphi;
	iwphi << w.x(), w.y(), w.z(), 0.   , 0.   , 0.,
					0.   , w.x(), 0    , w.y(), w.z(), 0.,
					0.   , 0.   , w.x(), 0.   , w.y(), w.z();
	Matrix3d hvphi = -vector3ToCrossMatrix(v);
	Matrix3d hwphi = vector3ToCrossMatrix(w);
	Matrix<double, 6, 10> mat;
	mat << Vector3d::Zero(), hvphi, iwphi,
				v, hwphi, Matrix<double, 3, 6>::Zero();
	return mat;
}


Eigen::Matrix<double, 10, 1> inertiaToVector(const sva::RBInertiad& rbi)
{
	Eigen::Matrix<double, 10, 1> vec;
	const Eigen::Vector3d& h = rbi.momentum();
	const Eigen::Matrix3d& I = rbi.inertia();
	vec << rbi.mass(),
				 h.x(), h.y(), h.z(),
				 I(0,0), I(0,1), I(0,2), I(1,1), I(1,2), I(2,2);
	return vec;
}


sva::RBInertiad vectorToInertia(const Eigen::Matrix<double, 10, 1>& vec)
{
	Eigen::Matrix3d I;
	I << vec(4), vec(5), vec(6),
			vec(5), vec(7), vec(8),
			vec(6), vec(8), vec(9);
	return sva::RBInertiad(vec(0), Eigen::Vector3d(vec(1), vec(2), vec(3)), I);
}


sva::RBInertiad sVectorToInertia(const Eigen::VectorXd& vec)
{
	if(vec.rows() != 10)
	{
		std::ostringstream str;
		str << "Vector size mismatch: expected size is 10 gived is "
				<< vec.rows();
		throw std::out_of_range(str.str());
	}
	return vectorToInertia(vec);
}


Eigen::VectorXd multiBodyToInertialVector(const rbd::MultiBody& mb)
{
	Eigen::VectorXd vec(mb.nrBodies()*10, 1);
	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		vec.segment(i*10, 10).noalias() = inertiaToVector(mb.body(i).inertia());
	}
	return vec;
}


IDIM::IDIM(const rbd::MultiBody& mb):
	Y_(Eigen::MatrixXd::Zero(mb.nrDof(), mb.nrBodies()*10))
{ }


void IDIM::computeY(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	const std::vector<Body>& bodies = mb.bodies();
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();

	Eigen::Matrix<double, 6, 10> bodyFPhi;
	for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
	{
		const sva::MotionVecd& vb_i = mbc.bodyVelB[i];
		Eigen::Matrix<double, 6, 10> vb_i_Phi(IMPhi(vb_i));

		bodyFPhi.noalias() = IMPhi(mbc.bodyAccB[i]);
		// bodyFPhi += vb_i x* IMPhi(vb_i)
		// is faster to convert each col in a ForceVecd
		// than using sva::vector6ToCrossDualMatrix
		for(int c = 0; c < 10; ++c)
		{
			bodyFPhi.col(c).noalias() +=
				(vb_i.crossDual(sva::ForceVecd(vb_i_Phi.col(c)))).vector();
		}

		int iDofPos = mb.jointPosInDof(i);

		Y_.block(iDofPos, i*10, joints[i].dof(), 10).noalias() =
			mbc.motionSubspace[i].transpose()*bodyFPhi;

		int j = i;
		while(pred[j] != -1)
		{
			const sva::PTransformd& X_p_j = mbc.parentToSon[j];
			// bodyFPhi = X_p_j^T bodyFPhi
			// is faster to convert each col in a ForceVecd
			// than using X_p_j.inv().dualMatrix()
			for(int c = 0; c < 10; ++c)
			{
				bodyFPhi.col(c) =
					X_p_j.transMul(sva::ForceVecd(bodyFPhi.col(c))).vector();
			}
			j = pred[j];

			int jDofPos = mb.jointPosInDof(j);
			if(joints[j].dof() != 0)
			{
				Y_.block(jDofPos, i*10, joints[j].dof(), 10).noalias() =
					mbc.motionSubspace[j].transpose()*bodyFPhi;
			}
		}
	}
}


void IDIM::sComputeY(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchParentToSon(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);
	checkMatchBodyAcc(mb, mbc);

	computeY(mb, mbc);
}

} // rbd
