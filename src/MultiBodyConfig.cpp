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
#include "RBDyn/MultiBodyConfig.h"

// includes
// std
#include <sstream>

// RBDyn
#include "RBDyn/MultiBody.h"

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
	gravity(0., 9.81, 0.)
{
	for(int i = 0; i < static_cast<int>(q.size()); ++i)
	{
		q[i].resize(mb.joint(i).params());
		alpha[i].resize(mb.joint(i).dof());
		alphaD[i].resize(mb.joint(i).dof());

		jointTorque[i].resize(mb.joint(i).dof());
		motionSubspace[i].resize(6, mb.joint(i).dof());
	}
}


void MultiBodyConfig::zero(const MultiBody& mb)
{
	for(int i = 0; i < static_cast<int>(q.size()); ++i)
	{
		q[i] = mb.joint(i).zeroParam();
		alpha[i] = mb.joint(i).zeroDof();
		alphaD[i] = mb.joint(i).zeroDof();

		jointTorque[i] = mb.joint(i).zeroDof();
	}

	for(std::size_t i = 0; i < force.size(); ++i)
	{
		force[i] = sva::ForceVecd(Eigen::Vector6d::Zero());
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


/**
	*													ConfigConverter
	*/


ConfigConverter::ConfigConverter(const MultiBody& from, const MultiBody& to):
  jInd_(from.nrJoints() - 1),
  bInd_(from.nrBodies())
{
  using namespace Eigen;

	const std::vector<Body>& bodies = from.bodies();
	const std::vector<Joint>& joints = from.joints();

	for(std::size_t i = 0; i < joints.size() - 1; ++i)
	{
		jInd_[i] = to.jointIndexByName(joints[i + 1].name());
	}

	for(std::size_t i = 0; i < bodies.size(); ++i)
	{
		bInd_[i] = to.bodyIndexByName(bodies[i].name());
	}
}


void ConfigConverter::convert(const MultiBodyConfig& from, MultiBodyConfig& to) const
{
	for(std::size_t i = 0; i < jInd_.size(); ++i)
	{
		to.q[jInd_[i]] = from.q[i + 1];
		to.alpha[jInd_[i]] = from.alpha[i + 1];
		to.alphaD[jInd_[i]] = from.alphaD[i + 1];
	}

	for(std::size_t i = 0; i < bInd_.size(); ++i)
	{
		to.force[bInd_[i]] = from.force[i];
	}
}


ConfigConverter* ConfigConverter::sConstructor(const MultiBody& from, const MultiBody& to)
{
	bool isOk = true;

	if(from.nrBodies() != to.nrBodies())
		isOk = false;

	if(from.nrJoints() != to.nrJoints())
		isOk = false;

	if(isOk)
	{
		const std::vector<Joint>& joints = from.joints();
		const std::unordered_map<std::string, int>& jI2Ito = to.jointIndexByName();

		for(const Joint& j: joints)
		{
			if(jI2Ito.find(j.name()) == jI2Ito.end())
			{
				isOk = false;
				break;
			}
		}
	}

	if(isOk)
	{
		const std::vector<Body>& bodies = from.bodies();
		const std::unordered_map<std::string, int>& bI2Ito = to.bodyIndexByName();

		for(const Body& b: bodies)
		{
			if(bI2Ito.find(b.name()) == bI2Ito.end())
			{
				isOk = false;
				break;
			}
		}
	}

	if(!isOk)
	{
		throw std::domain_error("MultiBody mismatch");
	}

	return new ConfigConverter(from, to);
}


void ConfigConverter::sConvert(const MultiBodyConfig& from, MultiBodyConfig& to) const
{
	bool isOk = true;

	if(from.q.size() != to.q.size())
		isOk = false;

	if(from.alpha.size() != to.alpha.size())
		isOk = false;

	if(from.alphaD.size() != to.alphaD.size())
		isOk = false;

	if(from.force.size() != to.force.size())
		isOk = false;

	if(!isOk)
	{
		throw std::domain_error("MultiBody mismatch");
	}

	convert(from, to);
}


/**
	*													Param convertion
	*/


void paramToVector(const std::vector<std::vector<double> >& v,
	Eigen::Ref<Eigen::VectorXd> e)
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

void sParamToVector(const std::vector<std::vector<double> >& v,
	Eigen::Ref<Eigen::VectorXd> e)
{
	int nb = 0;
	for(int i = 0; i < static_cast<int>(v.size()); ++i)
	{
		nb += static_cast<int>(v[i].size());
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



Eigen::VectorXd paramToVector(const MultiBody& mb,
	const std::vector<std::vector<double> >& v)
{
	Eigen::VectorXd e(mb.nrParams());
	paramToVector(v, e);

	return std::move(e);
}

Eigen::VectorXd sParamToVector(const MultiBody& mb,
	const std::vector<std::vector<double> >& v)
{
	if(static_cast<int>(v.size()) != mb.nrJoints())
	{
		std::ostringstream str;
		str << "Param vector size and MultiBody mismatch: expected size "
				<< mb.nrJoints() << " gived " << v.size();
		throw std::out_of_range(str.str());
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		if(mb.joint(i).params() != static_cast<int>(v[i].size()))
		{
			std::ostringstream str;
			str << "Parameters of joint " << i << " mismatch: expected size "
					<< mb.joint(i).params() << " gived " << v[i].size();
			throw std::out_of_range(str.str());
		}
	}

	return std::move(paramToVector(mb, v));
}



Eigen::VectorXd dofToVector(const MultiBody& mb,
	const std::vector<std::vector<double> >& v)
{
	Eigen::VectorXd e(mb.nrDof());
	paramToVector(v, e);

	return std::move(e);
}


Eigen::VectorXd sDofToVector(const MultiBody& mb,
	const std::vector<std::vector<double> >& v)
{
	if(static_cast<int>(v.size()) != mb.nrJoints())
	{
		std::ostringstream str;
		str << "Dof vector size and MultiBody mismatch: expected size "
				<< mb.nrJoints() << " gived " << v.size();
		throw std::out_of_range(str.str());
	}

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		if(mb.joint(i).dof() != static_cast<int>(v[i].size()))
		{
			std::ostringstream str;
			str << "Dof of joint " << i << " mismatch: expected size "
					<< mb.joint(i).dof() << " gived " << v[i].size();
			throw std::out_of_range(str.str());
		}
	}

	return std::move(dofToVector(mb, v));
}



void vectorToParam(const Eigen::Ref<const Eigen::VectorXd>& e,
	std::vector<std::vector<double> >& v)
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

void sVectorToParam(const Eigen::Ref<const Eigen::VectorXd>& e,
	std::vector<std::vector<double> >& v)
{
	int nb = 0;
	for(std::size_t i = 0; i < v.size(); ++i)
	{
		nb += static_cast<int>(v[i].size());
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



std::vector<std::vector<double> > vectorToParam(const MultiBody& mb,
	const Eigen::VectorXd& e)
{
	std::vector<std::vector<double>> ret(mb.nrJoints());
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		ret[i].resize(mb.joint(i).params());
	}

	vectorToParam(e, ret);

	return std::move(ret);
}


std::vector<std::vector<double> > sVectorToParam(const MultiBody& mb,
	const Eigen::VectorXd& e)
{
	if(mb.nrParams() != e.rows())
	{
		std::ostringstream str;
		str << "Parameter vector size mismatch: expected size "
				<< mb.nrParams() << " gived " << e.rows();
		throw std::out_of_range(str.str());
	}
	return std::move(vectorToParam(mb, e));
}



std::vector<std::vector<double> > vectorToDof(const MultiBody& mb,
	const Eigen::VectorXd& e)
{
	std::vector<std::vector<double>> ret(mb.nrJoints());
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		ret[i].resize(mb.joint(i).dof());
	}

	vectorToParam(e, ret);

	return std::move(ret);
}


std::vector<std::vector<double> > sVectorToDof(const MultiBody& mb,
	const Eigen::VectorXd& e)
{
	if(mb.nrDof() != e.rows())
	{
		std::ostringstream str;
		str << "Dof vector size mismatch: expected size "
				<< mb.nrDof() << " gived " << e.rows();
		throw std::out_of_range(str.str());
	}
	return std::move(vectorToDof(mb, e));
}



void checkMatchBodyPos(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodiesVector(mb, mbc.bodyPosW, "bodyPosW");
}


void checkMatchParentToSon(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchJointsVector(mb, mbc.parentToSon, "parentToSon");
}


void checkMatchBodyVel(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodiesVector(mb, mbc.bodyVelW, "bodyVelW");
	checkMatchBodiesVector(mb, mbc.bodyVelB, "bodyVelB");
}


void checkMatchBodyAcc(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchBodiesVector(mb, mbc.bodyAccB, "bodyAccB");
}


void checkMatchJointConf(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchJointsVector(mb, mbc.jointConfig, "jointConfig");
}


void checkMatchJointVelocity(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchJointsVector(mb, mbc.jointVelocity, "jointVelocity");
}


void checkMatchJointTorque(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	checkMatchJointsVector(mb, mbc.jointTorque, "jointTorque");

	for(int i = 0; i < static_cast<int>(mbc.jointTorque.size()); ++i)
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
	checkMatchJointsVector(mb, mbc.motionSubspace, "motionSubspace");

	for(int i = 0; i < static_cast<int>(mbc.motionSubspace.size()); ++i)
	{
		if(mbc.motionSubspace[i].cols() != static_cast<unsigned>(mb.joint(i).dof()))
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
	checkMatchJointsVector(mb, mbc.q, "Generalized position variable vector");

	for(int i = 0; i < static_cast<int>(mbc.q.size()); ++i)
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
	checkMatchJointsVector(mb, mbc.alpha, "Generalized velocity variable vector");

	for(int i = 0; i < static_cast<int>(mbc.alpha.size()); ++i)
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
	checkMatchJointsVector(mb, mbc.alphaD, "Generalized acceleration variable vector");

	for(int i = 0; i < static_cast<int>(mbc.alphaD.size()); ++i)
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
	checkMatchBodiesVector(mb, mbc.force, "External force vector");
}

} // namespace rbd
