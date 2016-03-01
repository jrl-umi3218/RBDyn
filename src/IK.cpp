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
#include "IK.h"

// includes
// RBDyn
#include "FK.h"
#include "FV.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"

//SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{

static const int MAX_IK_ITERATIONS = 50;
static const double LAMBDA = 0.9;
static const double IK_THRESHOLD = 1e-8;
static const double ALMOST_ZERO = 1e-8;

InverseKinematics::InverseKinematics(const MultiBody& mb, int ef_index):
	ef_index_(ef_index),
	jac_(mb, mb.body(ef_index).id()),
	svd_()
{
}

struct CwiseRoundOp {
	CwiseRoundOp(const double& inf, const double& sup) : m_inf(inf), m_sup(sup) {}
	double operator()(const double& x) const { return x>m_inf && x<m_sup ? 0 : x; }
	double m_inf, m_sup;
};

bool InverseKinematics::inverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
                                          const sva::PTransformd& ef_target)
{
	int iter = 0;
	bool converged = false;
	int dof = 0;
	rbd::forwardKinematics(mb, mbc);
	Eigen::MatrixXd jacMat;
	Eigen::Vector6d v = Eigen::Vector6d::Ones();
	Eigen::Vector3d rotErr;
	Eigen::VectorXd res = Eigen::VectorXd::Zero(3);
	while( ! converged && iter < MAX_IK_ITERATIONS)
	{
		jacMat = jac_.jacobian(mb, mbc);
		//non-strict zeros in jacobian can be a problem...
		double eps = ALMOST_ZERO;
		jacMat = jacMat.unaryExpr(CwiseRoundOp(-eps, eps));
		svd_.compute(jacMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		rotErr = sva::rotationError(mbc.bodyPosW[ef_index_].rotation(),
		                            ef_target.rotation());
		v << rotErr, ef_target.translation() - mbc.bodyPosW[ef_index_].translation();
		converged = v.norm() < IK_THRESHOLD;
		res = svd_.solve(v);

		dof = 0;
		for(auto index : jac_.jointsPath())
		{
			std::vector<double>& qi = mbc.q[index];
			for(auto &qv : qi)
			{
				qv += LAMBDA*res[dof];
				++dof;
			}
		}

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);
		iter++;
	}
	return converged;
}

bool InverseKinematics::sInverseKinematics(const MultiBody& mb, MultiBodyConfig& mbc,
					   const sva::PTransformd& ef_target)
{
	checkMatchQ(mb, mbc);
	checkMatchBodyPos(mb, mbc);
	checkMatchJointConf(mb, mbc);
	checkMatchParentToSon(mb, mbc);

	return inverseKinematics(mb, mbc, ef_target);
}

} // namespace rbd
