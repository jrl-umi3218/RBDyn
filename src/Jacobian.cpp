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
#include "Jacobian.h"

// includes
// std
#include <algorithm>
#include <stdexcept>

// RBDyn
#include "MultiBodyConfig.h"

namespace rbd
{

Jacobian::Jacobian()
{}

Jacobian::Jacobian(const MultiBody& mb, int bodyId, const Eigen::Vector3d& point):
  jointsPath_(),
  point_(point),
  jac_(),
  jacDot_()
{
  int index = mb.sBodyIndexById(bodyId);

	int dof = 0;
	while(index != -1)
	{
		jointsPath_.insert(jointsPath_.begin(), index);
		dof += mb.joint(index).dof();

		index = mb.parent(index);
	}

	jac_.resize(6, dof);
	jacDot_.resize(6, dof);
}

MultiBody Jacobian::subMultiBody(const MultiBody& mb) const
{
	std::vector<Body> bodies;
	std::vector<Joint> joints;

	std::vector<int> pred;
	std::vector<int> succ;
	std::vector<int> parent;
	std::vector<sva::PTransformd> Xt;

	for(int index = 0; index < static_cast<int>(jointsPath_.size()); ++index)
	{
		int i = jointsPath_[index];
		// body info
		bodies.push_back(mb.body(i));
		parent.push_back(index - 1);

		// joint info
		joints.push_back(mb.joint(i));
		succ.push_back(index);
		pred.push_back(index - 1);
		Xt.push_back(mb.transform(i));
	}

	return MultiBody(std::move(bodies), std::move(joints),
					std::move(pred), std::move(succ), std::move(parent), std::move(Xt));
}

const Eigen::MatrixXd&
Jacobian::jacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	int curJ = 0;
	int N = jointsPath_.back();

	sva::PTransformd X_Np = point_*mbc.bodyPosW[N];
	sva::PTransformd E_N_0(Eigen::Matrix3d(mbc.bodyPosW[N].rotation().transpose()));
	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];

		sva::PTransformd X_i_N = X_Np*mbc.bodyPosW[i].inv();

		jac_.block(0, curJ, 6, joints[i].dof()) =
			(E_N_0*X_i_N).matrix()*mbc.motionSubspace[i];

		curJ += joints[i].dof();
	}

	return jac_;
}


const Eigen::MatrixXd&
Jacobian::bodyJacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	int curJ = 0;
	int N = jointsPath_.back();

	sva::PTransformd X_Np = point_*mbc.bodyPosW[N];
	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];

		sva::PTransformd X_i_N = X_Np*mbc.bodyPosW[i].inv();

		jac_.block(0, curJ, 6, joints[i].dof()) = X_i_N.matrix()*mbc.motionSubspace[i];

		curJ += joints[i].dof();
	}

	return jac_;
}


const Eigen::MatrixXd& Jacobian::vectorBodyJacobian(const MultiBody& mb,
																								 const MultiBodyConfig& mbc,
																								 const Eigen::Vector3d& vector)
{
	const std::vector<Joint>& joints = mb.joints();

	int curJ = 0;
	int N = jointsPath_.back();

	sva::PTransformd X_Np = point_*mbc.bodyPosW[N];
	sva::PTransformd vec = sva::PTransformd(vector);
	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];

		sva::PTransformd X_i_N = X_Np*mbc.bodyPosW[i].inv();
		sva::PTransformd X_i_Nv = vec*X_i_N;
		Eigen::Vector3d diff(X_i_N.translation() - X_i_Nv.translation());

		// Compute translation component of : Jac_{Nv} - Jac_{N}
		// Iteration : {}^{Nv}X_i S_i - {}^NX_i S_i
		//             ({}^{Nv}T_i - {}^NT_i) S_i
		//             {}^NE_i(T) (({}^{N}T_i - {}^{Nv}T_i) \times W_i)
		jac_.block(3, curJ, 3, joints[i].dof()) =
			X_i_N.rotation()*(sva::vector3ToCrossMatrix(diff)*\
				mbc.motionSubspace[i].block(0, 0, 3, joints[i].dof()));

		curJ += joints[i].dof();
	}

	return jac_;
}


const Eigen::MatrixXd&
Jacobian::jacobianDot(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	int curJ = 0;
	int N = jointsPath_.back();

	sva::PTransformd X_0_Np = point_*mbc.bodyPosW[N];
	// speed of point in body N
	sva::MotionVecd X_VNp = point_*mbc.bodyVelB[N];

	sva::PTransformd E_N_0(Eigen::Matrix3d(mbc.bodyPosW[N].rotation().transpose()));
	// angular velocity of rotation N to O
	sva::MotionVecd E_VN(mbc.bodyVelW[N].angular(), Eigen::Vector3d::Zero());

	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];

		sva::PTransformd X_i_Np = X_0_Np*mbc.bodyPosW[i].inv();
		// speed of X_i_N in Np coordinate
		sva::MotionVecd X_VNp_i_Np = X_i_Np*mbc.bodyVelB[i] - X_VNp;

		for(int j = 0; j < joints[i].dof(); ++j)
		{
			sva::MotionVecd S_ij(mbc.motionSubspace[i].col(j));

			// JD_i = E_N_0_d*X_i_N*S_i + E_N_0*X_i_N_d*S_i
			// E_N_0_d = (ANG_VN)_0 x E_N_0
			// X_i_N_d = (Vi - VN)_N x X_i_N

			jacDot_.block<6, 1>(0, curJ) =
				(E_VN.cross(E_N_0*X_i_Np*S_ij) +
				E_N_0*X_VNp_i_Np.cross(X_i_Np*S_ij)).vector();
			++curJ;
		}
	}

	return jacDot_;
}


const Eigen::MatrixXd&
Jacobian::bodyJacobianDot(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	const std::vector<Joint>& joints = mb.joints();

	int curJ = 0;
	int N = jointsPath_.back();

	sva::PTransformd X_0_Np = point_*mbc.bodyPosW[N];
	// speed of point in body N
	sva::MotionVecd X_VNp = point_*mbc.bodyVelB[N];

	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];

		sva::PTransformd X_i_Np = X_0_Np*mbc.bodyPosW[i].inv();
		// speed of X_i_N in Np coordinate
		sva::MotionVecd X_VNp_i_Np = X_i_Np*mbc.bodyVelB[i] - X_VNp;

		for(int j = 0; j < joints[i].dof(); ++j)
		{
			sva::MotionVecd S_ij(mbc.motionSubspace[i].col(j));

			// JD_i = X_i_N_d*S_i
			// X_i_N_d = (Vi - VN)_N x X_i_N

			jacDot_.block<6, 1>(0, curJ) =
				(X_VNp_i_Np.cross(X_i_Np*S_ij)).vector();
			++curJ;
		}
	}

	return jacDot_;
}


void Jacobian::translateJacobian(const Eigen::MatrixXd& jac,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
	Eigen::MatrixXd& res)
{
	int N = jointsPath_.back();
	sva::PTransformd E_N_0(Eigen::Matrix3d(mbc.bodyPosW[N].rotation().transpose()));
	sva::PTransformd t(point);

	t = E_N_0*t*E_N_0.inv();

	for(int i = 0; i < jac.cols(); ++i)
	{
		sva::MotionVecd mv(jac.col(i));
		res.col(i) = (t*mv).vector();
	}
}


void Jacobian::translateBodyJacobian(const Eigen::MatrixXd& jac,
	const MultiBodyConfig& /* mbc */, const Eigen::Vector3d& point,
	Eigen::MatrixXd& res)
{
	sva::PTransformd t(point);

	for(int i = 0; i < jac.cols(); ++i)
	{
		sva::MotionVecd mv(jac.col(i));
		res.col(i) = (t*mv).vector();
	}
}


void Jacobian::fullJacobian(const MultiBody& mb,
	const Eigen::Ref<const Eigen::MatrixXd>& jac,
	Eigen::MatrixXd& res) const
{
	res.setZero();
	int jacPos = 0;
	for(std::size_t index = 0; index < jointsPath_.size(); ++index)
	{
		int i = jointsPath_[index];
		int dof = mb.joint(i).dof();
		res.block(0, mb.jointPosInDof(i), res.rows(), dof) =
			jac.block(0, jacPos, res.rows(), dof);
		jacPos += dof;
	}
}



const Eigen::MatrixXd&
Jacobian::sJacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return jacobian(mb, mbc);
}


const Eigen::MatrixXd&
Jacobian::sBodyJacobian(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return bodyJacobian(mb, mbc);
}


const Eigen::MatrixXd&
Jacobian::sVectorBodyJacobian(const MultiBody& mb, const MultiBodyConfig& mbc,
	const Eigen::Vector3d& vec)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return vectorBodyJacobian(mb, mbc, vec);
}


MultiBody Jacobian::sSubMultiBody(const MultiBody& mb) const
{
	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return subMultiBody(mb);
}


const Eigen::MatrixXd& Jacobian::sJacobianDot(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return jacobianDot(mb, mbc);
}


const Eigen::MatrixXd& Jacobian::sBodyJacobianDot(const MultiBody& mb,
	const MultiBodyConfig& mbc)
{
	checkMatchBodyPos(mb, mbc);
	checkMatchBodyVel(mb, mbc);
	checkMatchMotionSubspace(mb, mbc);

	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	return bodyJacobianDot(mb, mbc);
}


void Jacobian::sTranslateJacobian(const Eigen::MatrixXd& jac,
	const MultiBodyConfig& mbc, const Eigen::Vector3d& point,
	Eigen::MatrixXd& res)
{
	if(jointsPath_.back() >= static_cast<int>(mbc.bodyPosW.size()))
	{
		throw std::domain_error("jointsPath mismatch MultiBodyConfig");
	}

	if(jac.cols() != jac_.cols() || jac.rows() != jac_.rows())
	{
		std::ostringstream str;
		str << "jac matrix size mismatch: expected size ("
				<< jac_.rows() << " x " << jac_.cols() << ")" << " gived ("
				<< jac.rows() << " x " << jac.cols() << ")" ;
		throw std::domain_error(str.str());
	}

	if(res.cols() != jac_.cols() || res.rows() != jac_.rows())
	{
		std::ostringstream str;
		str << "res matrix size mismatch: expected size ("
				<< jac_.rows() << " x " << jac_.cols() << ")" << " gived ("
				<< res.rows() << " x " << res.cols() << ")" ;
		throw std::domain_error(str.str());
	}

	translateJacobian(jac, mbc, point, res);
}


void Jacobian::sFullJacobian(const MultiBody& mb, const Eigen::MatrixXd& jac,
	Eigen::MatrixXd& res) const
{
	int m = *std::max_element(jointsPath_.begin(), jointsPath_.end());
	if(m >= static_cast<int>(mb.nrJoints()))
	{
		throw std::domain_error("jointsPath mismatch MultiBody");
	}

	if(jac.cols() != jac_.cols() || jac.rows() != jac_.rows())
	{
		std::ostringstream str;
		str << "jac matrix size mismatch: expected size ("
				<< jac_.rows() << " x " << jac_.cols() << ")" << " gived ("
				<< jac.rows() << " x " << jac.cols() << ")" ;
		throw std::domain_error(str.str());
	}

	if(res.cols() != mb.nrDof() || res.rows() != 6)
	{
		std::ostringstream str;
		str << "res matrix size mismatch: expected size ("
				<< mb.nrDof() << " x " << "6 )" << " gived ("
				<< res.rows() << " x " << res.cols() << ")" ;
		throw std::domain_error(str.str());
	}

	fullJacobian(mb, jac, res);
}

} // namespace rbd

