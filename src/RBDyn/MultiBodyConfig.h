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

#pragma once

// includes
// std
#include <vector>
#include <stdexcept>

// sva
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "MultiBody.h"

#include <rbdyn/config.hh>

namespace rbd
{

struct RBDYN_DLLAPI MultiBodyConfig
{
	MultiBodyConfig()
	{}
	MultiBodyConfig(const MultiBody& mb);

	/// Set the multibody at zero configuration
	void zero(const MultiBody& mb);

	/// Generalized position variable.
	std::vector<std::vector<double>> q;

	/// Generalized speed variable.
	std::vector<std::vector<double>> alpha;

	/// Generalized acceleration variable.
	std::vector<std::vector<double>> alphaD;



	/// Total external force acting on each body in world coordinate.
	std::vector<sva::ForceVecd> force;

	/// Joints configuration (Xj).
	std::vector<sva::PTransformd> jointConfig;

	/// Joints velocity (Xj*j.motion()).
	std::vector<sva::MotionVecd> jointVelocity;

	/// Joints torque.
	std::vector<std::vector<double>> jointTorque;



	/// Motion subspace (Xj.j.subspace).
	std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> motionSubspace;



	/// Bodies transformation in world coordinate.
	std::vector<sva::PTransformd> bodyPosW;

	/// Transformation from parent(i) to i in body coordinate (Xj*Xt).
	std::vector<sva::PTransformd> parentToSon;



	/// Bodies speed in world coordinate.
	std::vector<sva::MotionVecd> bodyVelW;

	/// Bodies speed in Body coordinate.
	std::vector<sva::MotionVecd> bodyVelB;

	/// Bodies acceleration in Body coordinate.
	std::vector<sva::MotionVecd> bodyAccB;

	/// gravity acting on the multibody.
	Eigen::Vector3d gravity;


	// python binding function

	std::vector<Eigen::MatrixXd> python_motionSubspace();
	void python_motionSubspace(const std::vector<Eigen::MatrixXd>& v);
};



/**
	* Convert a MultiBodyConfig to another MultiBodyConfig of the same MultiBodyGraph.
	* This class only convert q, alpha, alphaD and force.
	*/
class RBDYN_DLLAPI ConfigConverter
{
public:
	ConfigConverter(const MultiBody& from, const MultiBody& to);

	void convert(const MultiBodyConfig& from, MultiBodyConfig& to) const;

	/**
		* Convert a vector representing joint data.
		* The first joint (base) is ignored.
		*/
	template<typename T>
	void convertJoint(const std::vector<T>& from, std::vector<T>& to) const;

	template<typename T>
	std::vector<T> convertJoint(const std::vector<T>& from) const;

	// safe version for python binding

	/** safe version of @see ConfigConverter.
		* @throw std::domain_error If mb don't match mbc.
		*/
	static ConfigConverter* sConstructor(const MultiBody& from, const MultiBody& to);

	/** safe version of @see convert.
		* @throw std::domain_error If mb don't match mbc.
		*/
	void sConvert(const MultiBodyConfig& from, MultiBodyConfig& to) const;

	/** safe version of @see convertJoint.
		* @throw std::domain_error If mb don't match mbc.
		*/
	template<typename T>
	void sConvertJoint(const std::vector<T>& from, std::vector<T>& to) const;

private:
	std::vector<int> jInd_;
	std::vector<int> bInd_;
};



/**
	* Convert parameter vector to Eigen Vector.
	* @param v Parameter vector.
	* @param e Output Eigen vector (must be of the good size).
	*/
RBDYN_DLLAPI void paramToVector(const std::vector<std::vector<double>>& v,
	Eigen::Ref<Eigen::VectorXd> e);

/**
	* Safe version of @see paramToVector.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
RBDYN_DLLAPI void sParamToVector(const std::vector<std::vector<double>>& v,
	Eigen::Ref<Eigen::VectorXd> e);



/**
	* Convert parameter vector to Eigen Vector.
	* @param mb MultiBody used has model.
	* @param v Parameter vector.
	* @return Parameter converted in eigen vector.
	*/
RBDYN_DLLAPI Eigen::VectorXd paramToVector(const MultiBody& mb,
	const std::vector<std::vector<double>>& v);

/**
	* Safe version of @see paramToVector.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
RBDYN_DLLAPI Eigen::VectorXd sParamToVector(const MultiBody& mb,
	const std::vector<std::vector<double>>& v);



/**
	* Convert dof vector to Eigen Vector.
	* @param mb MultiBody used has model.
	* @param v dof vector.
	* @return dof converted in eigen vector.
	*/
RBDYN_DLLAPI Eigen::VectorXd dofToVector(const MultiBody& mb,
	const std::vector<std::vector<double>>& v);

/**
	* Safe version of @see paramToDof.
	* @throw std::out_of_range if dof and Eigen vector mismatch.
	*/
RBDYN_DLLAPI Eigen::VectorXd sDofToVector(const MultiBody& mb,
	const std::vector<std::vector<double>>& v);



/**
	* Convert parameter vector to Eigen Vector.
	* @param e Eigen vector.
	* @param e Output Parameter vector (must be of the good size).
	*/
RBDYN_DLLAPI void vectorToParam(const Eigen::Ref<const Eigen::VectorXd>& e,
	std::vector<std::vector<double>>& v);

/**
	* Safe version of @see vectorToParam.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
RBDYN_DLLAPI void sVectorToParam(const Eigen::Ref<const Eigen::VectorXd>& e,
	std::vector<std::vector<double>>& v);



/**
	* Convert parameter vector to Eigen Vector.
	* @param mb MultiBody used has model.
	* @param e Eigen vector.
	* @return Eigen vector converted in parameter vector.
	*/
RBDYN_DLLAPI std::vector<std::vector<double>> vectorToParam(const MultiBody& mb,
	const Eigen::VectorXd& e);

/**
	* Safe version of @see vectorToParam.
	* @throw std::out_of_range if param and Eigen vector mismatch.
	*/
RBDYN_DLLAPI std::vector<std::vector<double>> sVectorToParam(const MultiBody& mb,
	const Eigen::VectorXd& e);



/**
	* Convert dof vector to Eigen Vector.
	* @param mb MultiBody used has model.
	* @param e Eigen vector.
	* @return Eigen vector converted in parameter vector.
	*/
RBDYN_DLLAPI std::vector<std::vector<double>> vectorToDof(const MultiBody& mb,
	const Eigen::VectorXd& e);

/**
	* Safe version of @see vectorToDof.
	* @throw std::out_of_range if dof and Eigen vector mismatch.
	*/
RBDYN_DLLAPI std::vector<std::vector<double>> sVectorToDof(const MultiBody& mb,
	const Eigen::VectorXd& e);



/// @throw std::domain_error If there is a mismatch between mb.nrBodies and vec.size()
template <typename T>
void checkMatchBodiesVector(const MultiBody& mb, const std::vector<T>& vec,
	const std::string& name);

/// @throw std::domain_error If there is a mismatch between mb.nrJoints and vec.size()
template <typename T>
void checkMatchJointsVector(const MultiBody& mb, const std::vector<T>& vec,
	const std::string& name);

/// @throw std::domain_error If there is a mismatch between mb and mbc.bodyPosW.
RBDYN_DLLAPI void checkMatchBodyPos(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.parentToSon.
RBDYN_DLLAPI void checkMatchParentToSon(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and
/// (mbc.bodyVelW, mbc.bodyVelB).
RBDYN_DLLAPI void checkMatchBodyVel(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.bodyAccB.
RBDYN_DLLAPI void checkMatchBodyAcc(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.jointConfig.
RBDYN_DLLAPI void checkMatchJointConf(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.jointVelocity.
RBDYN_DLLAPI void checkMatchJointVelocity(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.jointTorque.
RBDYN_DLLAPI void checkMatchJointTorque(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.motionSubspace.
RBDYN_DLLAPI void checkMatchMotionSubspace(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.q.
RBDYN_DLLAPI void checkMatchQ(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.alpha.
RBDYN_DLLAPI void checkMatchAlpha(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.alphaD.
RBDYN_DLLAPI void checkMatchAlphaD(const MultiBody& mb, const MultiBodyConfig& mbc);

/// @throw std::domain_error If there is a mismatch between mb and mbc.force.
RBDYN_DLLAPI void checkMatchForce(const MultiBody& mb, const MultiBodyConfig& mbc);



template <typename T>
void checkMatchBodiesVector(const MultiBody& mb, const std::vector<T>& vec,
	const std::string& name)
{
	if(int(vec.size()) != mb.nrBodies())
	{
		std::ostringstream str;
		str << name << " size mismatch: expected size "
				<< mb.nrBodies() << " gived " << vec.size();
		throw std::domain_error(str.str());
	}
}


template <typename T>
void checkMatchJointsVector(const MultiBody& mb, const std::vector<T>& vec,
	const std::string& name)
{
	if(int(vec.size()) != mb.nrJoints())
	{
		std::ostringstream str;
		str << name << " size mismatch: expected size "
				<< mb.nrJoints() << " gived " << vec.size();
		throw std::domain_error(str.str());
	}
}



template<typename T>
inline void
ConfigConverter::convertJoint(const std::vector<T>& from, std::vector<T>& to) const
{
	for(std::size_t i = 0; i < jInd_.size(); ++i)
	{
		to[jInd_[i]] = from[i + 1];
	}
}



template<typename T>
inline void
ConfigConverter::sConvertJoint(const std::vector<T>& from, std::vector<T>& to) const
{
	if(from.size() != to.size())
	{
		throw std::domain_error("from and to vector must have the same size");
	}

	convertJoint(from, to);
}



template<typename T>
inline std::vector<T>
ConfigConverter::convertJoint(const std::vector<T>& from) const
{
	std::vector<T> to(from.size());
	for(std::size_t i = 0; i < jInd_.size(); ++i)
	{
		to[jInd_[i]] = from[i + 1];
	}

	return std::move(to);
}


} // namespace rbd
