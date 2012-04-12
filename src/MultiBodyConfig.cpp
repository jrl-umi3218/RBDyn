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
#include "MultiBodyConfig.h"

// includes
// std
#include <sstream>
#include <stdexcept>

// RBDyn
#include "MultiBody.h"

namespace rbd
{
MultiBodyConfig::MultiBodyConfig(const MultiBody& mb):
	q(mb.nrJoints()),
	alpha(mb.nrJoints()),
	alphaD(mb.nrJoints()),
	force(mb.nrBodies()),
	jointConfig(mb.nrJoints()),
	jointVelocity(mb.nrJoints()),
	jointTorque(mb.nrJoints()),
	motionSubspace(mb.nrJoints()),
	bodyPosW(mb.nrBodies()),
	parentToSon(mb.nrBodies()),
	bodyVelW(mb.nrBodies()),
	bodyVelB(mb.nrBodies()),
	bodyAccB(mb.nrBodies()),
	gravity(0., -9.81, 0.)
{
	for(std::size_t i = 0; i < q.size(); ++i)
	{
		q[i].resize(mb.joint(i).params());
		alpha[i].resize(mb.joint(i).dof());
		alphaD[i].resize(mb.joint(i).dof());

		jointTorque[i].resize(mb.joint(i).dof());
		motionSubspace[i].resize(6, mb.joint(i).dof());
	}
}


std::vector<Eigen::MatrixXd> MultiBodyConfig::python_motionSubspace()
{
	std::vector<Eigen::MatrixXd> ret(motionSubspace.size());

	for(std::size_t i = 0; i < ret.size(); ++i)
	{
		ret[i] = motionSubspace[i];
	}

	return std::move(ret);
}


void MultiBodyConfig::python_motionSubspace(const std::vector<Eigen::MatrixXd>& v)
{
	motionSubspace.resize(v.size());
	for(std::size_t i = 0; i < v.size(); ++i)
	{
		motionSubspace[i] = v[i];
	}
}


void paramToVector(const std::vector<std::vector<double>>& v, Eigen::VectorXd& e)
{
	int pos = 0;
	for(auto& inV: v)
	{
		for(double d: inV)
		{
			e(pos) = d;
			++pos;
		}
	}
}

void sParamToVector(const std::vector<std::vector<double>>& v, Eigen::VectorXd& e)
{
	int nb = 0;
	for(std::size_t i = 0; i < v.size(); ++i)
	{
		nb += v[i].size();
	}

	if(nb != e.rows())
	{
		std::ostringstream str;
		str << "param vector size and eigen vector size mismatch: expected size "
				<< nb << " gived " << e.rows();
		throw std::out_of_range(str.str());
	}

	paramToVector(v, e);
}



void vectorToParam(const Eigen::VectorXd& e, std::vector<std::vector<double>>& v)
{
	int pos = 0;
	for(auto& inV: v)
	{
		for(double& d: inV)
		{
			d = e(pos);
			++pos;
		}
	}
}

void sVectorToParam(const Eigen::VectorXd& e, std::vector<std::vector<double>>& v)
{
	int nb = 0;
	for(std::size_t i = 0; i < v.size(); ++i)
	{
		nb += v[i].size();
	}

	if(nb != e.rows())
	{
		std::ostringstream str;
		str << "param vector size and eigen vector size mismatch: expected size "
				<< e.rows() << " gived " << nb;
		throw std::out_of_range(str.str());
	}

	vectorToParam(e, v);
}



void checkMatchBodyPos(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.bodyPosW.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "bodyPosW size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.bodyPosW.size();
		throw std::domain_error(str.str());
	}
}



void checkMatchParentToSon(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.parentToSon.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "parentToSon size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.parentToSon.size();
		throw std::domain_error(str.str());
	}

}


void checkMatchBodyVel(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.bodyVelW.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "bodyVelW size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.bodyVelW.size();
		throw std::domain_error(str.str());
	}

	if(mbc.bodyVelB.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "bodyVelB size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.bodyVelB.size();
		throw std::domain_error(str.str());
	}
}



void checkMatchBodyAcc(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.bodyAccB.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "bodyAccB size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.bodyAccB.size();
		throw std::domain_error(str.str());
	}
}



void checkMatchJointConf(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.jointConfig.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "jointConfig size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.jointConfig.size();
		throw std::domain_error(str.str());
	}
}



void checkMatchJointVelocity(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.jointVelocity.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "jointVelocity size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.jointVelocity.size();
		throw std::domain_error(str.str());
	}
}



void checkMatchJointTorque(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.jointTorque.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "jointTorque vector size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.jointTorque.size();
		throw std::domain_error(str.str());
	}

	for(std::size_t i = 0; i < mbc.jointTorque.size(); ++i)
	{
		if(mbc.jointTorque[i].size() != static_cast<std::size_t>(mb.joint(i).dof()))
		{
			std::ostringstream str;
			str << "Bad number of torque variable for Joint "
					<< mb.joint(i) << " at position " << i << ": expected size "
					<< mb.joint(i).dof() << " gived " << mbc.jointTorque[i].size();
			throw std::domain_error(str.str());
		}
	}
}



void checkMatchMotionSubspace(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.motionSubspace.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "motionSubspace vector size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.motionSubspace.size();
		throw std::domain_error(str.str());
	}

	for(std::size_t i = 0; i < mbc.motionSubspace.size(); ++i)
	{
		if(mbc.motionSubspace[i].cols() != static_cast<std::size_t>(mb.joint(i).dof()))
		{
			std::ostringstream str;
			str << "Bad motionSubspace matrix size for Joint "
					<< mb.joint(i) << " at position " << i << ": expected column number "
					<< mb.joint(i).dof() << " gived " << mbc.motionSubspace[i].cols();
			throw std::domain_error(str.str());
		}
	}
}



void checkMatchQ(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.q.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "Generalized position variable vector size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.q.size();
		throw std::domain_error(str.str());
	}

	for(std::size_t i = 0; i < mbc.q.size(); ++i)
	{
		if(mbc.q[i].size() != static_cast<std::size_t>(mb.joint(i).params()))
		{
			std::ostringstream str;
			str << "Bad number of generalized position variable for Joint "
					<< mb.joint(i) << " at position " << i << ": expected size "
					<< mb.joint(i).params() << " gived " << mbc.q[i].size();
			throw std::domain_error(str.str());
		}
	}
}



void checkMatchAlpha(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.alpha.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "Generalized velocity variable vector size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.alpha.size();
		throw std::domain_error(str.str());
	}

	for(std::size_t i = 0; i < mbc.alpha.size(); ++i)
	{
		if(mbc.alpha[i].size() != static_cast<std::size_t>(mb.joint(i).dof()))
		{
			std::ostringstream str;
			str << "Bad number of generalized velocity variable for Joint "
					<< mb.joint(i) << " at position " << i << ": expected size "
					<< mb.joint(i).dof() << " gived " << mbc.alpha[i].size();
			throw std::domain_error(str.str());
		}
	}
}



void checkMatchAlphaD(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.alphaD.size() != mb.nrJoints())
	{
		std::ostringstream str;
		str << "Generalized acceleration variable vector size mismatch: expected size "
				<< mb.nrJoints() << " gived " << mbc.alphaD.size();
		throw std::domain_error(str.str());
	}

	for(std::size_t i = 0; i < mbc.alphaD.size(); ++i)
	{
		if(mbc.alphaD[i].size() != static_cast<std::size_t>(mb.joint(i).dof()))
		{
			std::ostringstream str;
			str << "Bad number of generalized acceleration variable for Joint "
					<< mb.joint(i) << " at position " << i << ": expected size "
					<< mb.joint(i).dof() << " gived " << mbc.alphaD[i].size();
			throw std::domain_error(str.str());
		}
	}
}



void checkMatchForce(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.force.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "External force vector size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.force.size();
		throw std::domain_error(str.str());
	}
}

} // namespace rbd
