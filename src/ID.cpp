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
#include "ID.h"

// includes
// RBDyn
#include "MultiBody.h"
#include "MultiBodyConfig.h"

namespace rbd
{

InverseDynamics::InverseDynamics(const MultiBody& mb):
	f_(mb.nrBodies())
{ }

void InverseDynamics::inverseDynamics(const MultiBody& mb, MultiBodyConfig& mbc)
{
	const std::vector<Body>& bodies = mb.bodies();
	const std::vector<Joint>& joints = mb.joints();
	const std::vector<int>& pred = mb.predecessors();
	const std::vector<sva::PTransform>& Xf = mb.transformsFrom();
	const std::vector<sva::PTransform>& Xt = mb.transformsTo();

	sva::MotionVec a_0(Eigen::Vector3d::Zero(), mbc.gravity);

	for(std::size_t i = 0; i < bodies.size(); ++i)
	{
		const sva::PTransform& X_i = mbc.jointConfig[i];

		sva::PTransform X_p_i = Xt[i]*X_i*Xf[i];
		sva::PTransform X_j_i = Xt[i]*X_i;

		sva::MotionVec vj_i = X_j_i*joints[i].motion(mbc.alpha[i]);
		sva::MotionVec ai_tan = X_j_i*joints[i].tanAccel(mbc.alphaD[i]);

		const sva::MotionVec& vb_i = mbc.bodyVelB[i];

		if(pred[i] != -1)
			mbc.bodyAccB[i] = X_p_i*mbc.bodyAccB[pred[i]] + ai_tan + vb_i.cross(vj_i);
		else
			mbc.bodyAccB[i] = a_0 + ai_tan + vb_i.cross(vj_i);

		f_[i] = bodies[i].inertia()*mbc.bodyAccB[i] +
			vb_i.crossDual(bodies[i].inertia()*vb_i) -
			mbc.bodyPosW[i].dualMul(mbc.force[i]);
	}

	for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
	{
		const sva::PTransform& X_i = mbc.jointConfig[i];
		Eigen::MatrixXd S_i = X_i.matrix()*joints[i].motionSubspace();
		Eigen::Vector6d fj_i = (Xt[i].transMul(f_[i])).vector();

		for(int j = 0; j < joints[i].dof(); ++j)
		{
			mbc.jointTorque[i][j] = S_i.col(j).transpose()*fj_i;
		}

		if(pred[i] != -1)
		{
			sva::PTransform X_p_i = Xt[i]*X_i*Xf[i];
			f_[pred[i]] = f_[pred[i]] + X_p_i.transMul(f_[i]);
		}
	}
}

} // namespace rbd
