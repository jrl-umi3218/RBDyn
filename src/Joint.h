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
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{


/**
	* Utility function to compute a rotation matrix from the parameter vector.
	* @param q parameter vector with a least 4 values.
	* @return Rotation matrix in successor frame.
	*/
Eigen::Matrix3d QuatToE(const std::vector<double>& q);


/**
	* Joint representation.
	* Hold joint id and name and compute transformation, speed and motion
	* subspace.
	* WARNING : Spherical joint translation parameters are in FP coordinate.
	*/
class Joint
{
public:
	/// Joint type.
	enum Type
	{
		Rev, ///< Revolute joint about an user specified axis.
		Prism, ///< Prismatique joint about an user specified axis.
		Spherical, ///< Spherical joint, represented by a quaternion.
		Free, ///< Free joint, represented by a quaternion.
		Fixed ///< Fixed joint.
	};

	/// Old joint type for Api compatibility
	enum OldType
	{
		RevX, ///< Revolute joint about X axis.
		RevY, ///< Revolute joint about Y axis.
		RevZ, ///< Revolute joint about Z axis.
		PrismX, ///< Prismatique joint about X axis.
		PrismY, ///< Prismatique joint about Y axis.
		PrismZ ///< Prismatique joint about Z axis.
	};

public:
	Joint()
	{}

	/**
		* Compatibility constructor
		* @param type Joint type.
		* @param forward Joint is in forward direction if true.
		* @param id Joint id, must be unique in a multibody.
		* @param name Joint name.
		*/
	Joint(OldType type, bool forward, int id, std::string name);

	/**
		* @param type Joint type.
		* @param axis User specified.
		* @param forward Joint is in forward direction if true.
		* @param id Joint id, must be unique in a multibody.
		* @param name Joint name.
		*/
	Joint(Type type, const Eigen::Vector3d& axis, bool forward, int id,
		std::string name);

	/**
		* @param type Joint type.
		* @param forward Joint is in forward direction if true.
		* @param id Joint id, must be unique in a multibody.
		* @param name Joint name.
		*/
	Joint(Type type, bool forward, int id,
		std::string name);

	/// @return Joint type.
	Type type() const
	{
		return type_;
	}

	/// @return Joint direction.
	double direction() const
	{
		return dir_;
	}

	/// @return true if joint is forward, else false.
	bool forward() const
	{
		return dir_ == 1.;
	}

	/// @param forward Put the joint in forward direction if true.
	void forward(bool forward)
	{
		dir_ = forward ? 1. : -1;
		S_ *= dir_;
	}

	/**
		* Number of parameters of the joint.
		* @return Number of generalized position variable.
		*/
	int params() const
	{
		return params_;
	}

	/**
		* Number of degree of freedom of the joint.
		* @return Number of generalized speed and acceleration variable.
		*/
	int dof() const
	{
		return dof_;
	}

	/// @return Joint id.
	int id() const
	{
		return id_;
	}

	/// @return Joint name.
	const std::string& name() const
	{
		return name_;
	}

	/// @return Joint motion subspace in successor frame coordinate.
	const Eigen::Matrix<double, 6, Eigen::Dynamic>& motionSubspace() const
	{
		return S_;
	}

	/**
		* Compute the joint transformation from predecessor to successor frame.
		* @param q vector of generalized position variable.
		* @return Spatial transformation from predecessor to successor frame.
		*/
	sva::PTransform pose(const std::vector<double>& q) const;

	/**
		* Compute the joint velocity.
		* @param alpha vector of generalized speed variable.
		* @return Spatial motion vector of the joint.
		*/
	sva::MotionVec motion(const std::vector<double>& alpha) const;

	/**
		* Compute the tangential part of the acceleration S*alphaD.
		* @param alphaD vector of generalized acceleration variable.
		* @return Tangential part of acceleration
		*/
	sva::MotionVec tanAccel(const std::vector<double>& alphaD) const;

	/**
		* Safe version of pose method.
		* @see pose
		* @throw std::domain_error If the number of generalized position variable is
		* wrong.
		*/
	sva::PTransform sPose(const std::vector<double>& q) const;

	/**
		* Safe version of motion method.
		* @see motion
		* @throw std::domain_error If the number of generalized speed variable is
		* wrong.
		*/
	sva::MotionVec sMotion(const std::vector<double>& alpha) const;

	/**
		* Safe version of tanAccel method.
		* @see tanAccel
		* @throw std::domain_error If the number of generalized acceleration variable is
		* wrong.
		*/
	sva::MotionVec sTanAccel(const std::vector<double>& alphaD) const;

	bool operator==(const Joint& b) const
	{
		return id_ == b.id_ && name_ == b.name_;
	}

	bool operator!=(const Joint& b) const
	{
		return id_ != b.id_ || name_ != b.name_;
	}

private:
	void constructJoint(Type t, const Eigen::Vector3d& a);

private:
	Type type_;
	Eigen::Matrix<double, 6, Eigen::Dynamic> S_;
	double dir_; ///< joint direction

	int params_;
	int dof_;

	int id_;
	std::string name_;
};


inline std::ostream& operator<<(std::ostream& out, const Joint& b)
{
	out << "Joint: " << b.id() << ", " << b.name();
	return out;
}


inline Joint::Joint(OldType type, bool forward, int id, std::string name):
	dir_(forward ? 1. : -1),
	id_(id),
	name_(name)
{
	using namespace Eigen;

	switch(type)
	{
		case RevX:
			constructJoint(Rev, Vector3d::UnitX());
			break;
		case RevY:
			constructJoint(Rev, Vector3d::UnitY());
			break;
		case RevZ:
			constructJoint(Rev, Vector3d::UnitZ());
			break;
		case PrismX:
			constructJoint(Prism, Vector3d::UnitX());
			break;
		case PrismY:
			constructJoint(Prism, Vector3d::UnitY());
			break;
		case PrismZ:
			constructJoint(Prism, Vector3d::UnitZ());
			break;
		default:
			constructJoint(Fixed, Vector3d::Zero());
			break;
	}
}


inline Joint::Joint(Type type, const Eigen::Vector3d& axis,
	bool forward, int id, std::string name):
	dir_(forward ? 1. : -1),
	id_(id),
	name_(name)
{
	constructJoint(type, axis);
}


inline Joint::Joint(Type type,	bool forward, int id, std::string name):
	dir_(forward ? 1. : -1),
	id_(id),
	name_(name)
{
	constructJoint(type, Eigen::Vector3d::UnitZ());
}


inline sva::PTransform Joint::pose(const std::vector<double>& q) const
{
	using namespace Eigen;
	using namespace sva;
	Matrix3d rot;
	switch(type_)
	{
		case Rev:
			// minus S because rotation is anti trigonometric
			return PTransform(AngleAxisd(q[0], -S_.block<3, 1>(0, 0)).matrix());
		case Prism:
			return PTransform(Vector3d(S_.block<3, 1>(3, 0)*q[0]));
		case Spherical:
			return PTransform(Quaterniond(q[0], dir_*q[1], dir_*q[2], dir_*q[3]));
		case Free:
			rot = QuatToE(q);
			if(dir_ == 1.)
			{
				return PTransform(rot,
					Vector3d(q[4], q[5], q[6]));
			}
			else
			{
				return PTransform(rot,
					Vector3d(q[4], q[5], q[6])).inv();
			}
		case Fixed:
		default:
			return PTransform::Identity();
	}
}


inline sva::MotionVec Joint::motion(const std::vector<double>& alpha) const
{
	using namespace Eigen;
	using namespace sva;
	switch(type_)
	{
		case Rev:
			return MotionVec((Vector6d() << S_.block<3, 1>(0, 0)*alpha[0],
																			 Vector3d::Zero()).finished());
		case Prism:
			return MotionVec((Vector6d() << Vector3d::Zero(),
																			 S_.block<3, 1>(3, 0)*alpha[0]).finished());
		case Spherical:
			return MotionVec(S_*Vector3d(alpha[0], alpha[1], alpha[2]));
		case Free:
			return MotionVec(S_*(Vector6d() << alpha[0], alpha[1], alpha[2],
								alpha[3], alpha[4], alpha[5]).finished());
		case Fixed:
		default:
			return MotionVec(Vector6d::Zero());
	}
}


inline sva::MotionVec Joint::tanAccel(const std::vector<double>& alphaD) const
{
	using namespace Eigen;
	using namespace sva;
	switch(type_)
	{
		case Rev:
			return MotionVec((Vector6d() << S_.block<3, 1>(0, 0)*alphaD[0],
																			 Vector3d::Zero()).finished());
		case Prism:
			return MotionVec((Vector6d() << Vector3d::Zero(),
																			 S_.block<3, 1>(3, 0)*alphaD[0]).finished());
		case Spherical:
			return MotionVec(S_*Vector3d(alphaD[0], alphaD[1], alphaD[2]));
		case Free:
			return MotionVec(S_*(Vector6d() << alphaD[0], alphaD[1], alphaD[2],
								alphaD[3], alphaD[4], alphaD[5]).finished());
		case Fixed:
		default:
			return MotionVec(Vector6d::Zero());
	}
}


inline sva::PTransform Joint::sPose(const std::vector<double>& q) const
{
	if(q.size() != static_cast<unsigned int>(params_))
	{
		std::ostringstream str;
		str << "Wrong number of generalized position variable: expected " <<
					 params_ << " gived " << q.size();
		throw std::domain_error(str.str());
	}
	return pose(q);
}


inline sva::MotionVec Joint::sMotion(const std::vector<double>& alpha) const
{
	if(alpha.size() != static_cast<unsigned int>(dof_))
	{
		std::ostringstream str;
		str << "Wrong number of generalized speed variable: expected " <<
					 params_ << " gived " << alpha.size();
		throw std::domain_error(str.str());
	}
	return motion(alpha);
}


inline sva::MotionVec Joint::sTanAccel(const std::vector<double>& alphaD) const
{
	if(alphaD.size() != static_cast<unsigned int>(dof_))
	{
		std::ostringstream str;
		str << "Wrong number of generalized acceleration variable: expected " <<
					 params_ << " gived " << alphaD.size();
		throw std::domain_error(str.str());
	}
	return tanAccel(alphaD);
}


inline void Joint::constructJoint(Type t, const Eigen::Vector3d& a)
{
	using namespace Eigen;
	type_ = t;

	switch(t)
	{
		case Rev:
			S_ = dir_*(Vector6d() << a, Vector3d::Zero()).finished();
			params_ = 1;
			dof_ = 1;
			break;
		case Prism:
			S_ = dir_*(Vector6d() << Vector3d::Zero(), a).finished();
			params_ = 1;
			dof_ = 1;
			break;
		case Spherical:
			S_ = Matrix<double, 6, 3>::Zero();
			S_.block<3, 3>(0, 0).setIdentity();
			S_ *= dir_;
			params_ = 4;
			dof_ = 3;
			break;
		case Free:
			S_ = dir_*Matrix6d::Identity();
			params_ = 7;
			dof_ = 6;
			break;
		case Fixed:
		default:
			S_ = Matrix<double, 6, 0>::Zero();
			params_ = 0;
			dof_ = 0;
			break;
	}
}


inline Eigen::Matrix3d QuatToE(const std::vector<double>& q)
{
	using namespace Eigen;
	double p0 = q[0];
	double p1 = q[1];
	double p2 = q[2];
	double p3 = q[3];

	double p0p1 = p0*p1;
	double p0p2 = p0*p2;
	double p0p3 = p0*p3;

	double p1p2 = p1*p2;
	double p1p3 = p1*p3;

	double p2p3 = p2*p3;

	double p0s = std::pow(p0, 2);
	double p1s = std::pow(p1, 2);
	double p2s = std::pow(p2, 2);
	double p3s = std::pow(p3, 2);

	return 2.*(Matrix3d() << p0s + p1s - 0.5, p1p2 + p0p3, p1p3 - p0p2,
														p1p2 - p0p3, p0s + p2s - 0.5, p2p3 + p0p1,
														p1p3 + p0p2, p2p3 - p0p1, p0s + p3s - 0.5).finished();
}

} // namespace rbd
