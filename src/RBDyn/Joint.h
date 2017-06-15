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
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{


/**
	* Utility function to compute a rotation matrix from the parameter vector.
	* @param q parameter vector with a least 4 values (treated as wxyz).
	* @return Rotation matrix in successor frame.
	*/
template<typename T>
Eigen::Matrix3<T> QuatToE(const std::vector<T>& q);


/**
	* Joint representation.
	* Hold joint name (used as identifier) and compute transformation, speed and motion
	* subspace.
	* WARNING : Free joint translation parameters are in FP coordinate (Example 4.5 p81).
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
		Planar, ///< Planar joint (2 prismatic(X, Y) and 1 revolute(Z)).
		Cylindrical, ///< Cylindrical joint (Z prismatic, Z revolute).
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
	Joint();

	/**
		* Compatibility constructor
		* @param type Joint type.
		* @param forward Joint is in forward direction if true.
		* @param name Joint name, must be unique in a multibody.
		*/
	Joint(OldType type, bool forward, std::string name);

	/**
		* @param type Joint type.
		* @param axis User specified.
		* @param forward Joint is in forward direction if true.
		* @param name Joint name must be unique in a multibody.
		*/
	Joint(Type type, const Eigen::Vector3d& axis, bool forward,
		std::string name);

	/**
		* @param type Joint type.
		* @param forward Joint is in forward direction if true.
		* @param name Joint name, must be unique in a multibody.
		*/
	Joint(Type type, bool forward, std::string name);

	/**
		* Creates a fixed joint
		* @param name Joint name, must be unique in a multibody.
		*/
	Joint(std::string name);

	/**
	 * Make the joint mimic and specify mimic information.
	 *
	 * This change cannot be reversed.
	 *
	 * @param name Name of the joint that will be mimiced.
	 * @param multiplier Mimic multiplier
	 * @param offset Mimic offset
	 */
	void makeMimic(const std::string & name, double multiplier, double offset);

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

	/// @return Joint name.
	const std::string& name() const
	{
		return name_;
	}

	/// @return True if the joint is a mimic joint
	bool isMimic() const
	{
		return isMimic_;
	}

	/// @return Mimiced name
	const std::string& mimicName() const
	{
		return mimicName_;
	}

	/// @return Mimic multiplier
	double mimicMultiplier() const
	{
		return mimicMultiplier_;
	}

	/// @return Mimic offset
	double mimicOffset() const
	{
		return mimicOffset_;
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
	template<typename T>
	sva::PTransform<T> pose(const std::vector<T>& q) const;

	/**
		* Compute the joint velocity.
		* @param alpha vector of generalized speed variable.
		* @return Spatial motion vector of the joint.
		*/
	sva::MotionVecd motion(const std::vector<double>& alpha) const;

	/**
		* Compute the tangential part of the acceleration S*alphaD.
		* @param alphaD vector of generalized acceleration variable.
		* @return Tangential part of acceleration
		*/
	sva::MotionVecd tanAccel(const std::vector<double>& alphaD) const;

	/**
		* @return Joint configuation at zero.
		*/
	std::vector<double> zeroParam() const;

	/**
		* @return Joint velocity at zero.
		*/
	std::vector<double> zeroDof() const;

	/**
		* Safe version of pose method.
		* @see pose
		* @throw std::domain_error If the number of generalized position variable is
		* wrong.
		*/
	sva::PTransformd sPose(const std::vector<double>& q) const;

	/**
		* Safe version of motion method.
		* @see motion
		* @throw std::domain_error If the number of generalized speed variable is
		* wrong.
		*/
	sva::MotionVecd sMotion(const std::vector<double>& alpha) const;

	/**
		* Safe version of tanAccel method.
		* @see tanAccel
		* @throw std::domain_error If the number of generalized acceleration variable is
		* wrong.
		*/
	sva::MotionVecd sTanAccel(const std::vector<double>& alphaD) const;

	bool operator==(const Joint& b) const
	{
		return name_ == b.name_;
	}

	bool operator!=(const Joint& b) const
	{
		return name_ != b.name_;
	}

public:
	/**
		* @return Joint configuation at zero.
		*/
	static std::vector<double> ZeroParam(Type type);

	/**
		* @return Joint velocity at zero.
		*/
	static std::vector<double> ZeroDof(Type type);

private:
	void constructJoint(Type t, const Eigen::Vector3d& a);

private:
	Type type_;
	Eigen::Matrix<double, 6, Eigen::Dynamic> S_;
	double dir_; ///< joint direction

	int params_;
	int dof_;

	std::string name_;

	bool isMimic_ = false;
	std::string mimicName_ = "";
	double mimicMultiplier_ = 1.0;
	double mimicOffset_ = 0.0;
};


inline std::ostream& operator<<(std::ostream& out, const Joint& b)
{
	out << "Joint: " << b.name();
	return out;
}

inline Joint::Joint() : dir_(0.0), params_(0), dof_(0), name_("") {}

inline Joint::Joint(OldType type, bool forward, std::string name):
	dir_(forward ? 1. : -1),
	params_(0),
	dof_(0),
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
	bool forward, std::string name):
	dir_(forward ? 1. : -1),
	params_(0),
	dof_(0),
	name_(name)
{
	constructJoint(type, axis);
}


inline Joint::Joint(Type type,	bool forward, std::string name):
	dir_(forward ? 1. : -1),
	params_(0),
	dof_(0),
	name_(name)
{
	constructJoint(type, Eigen::Vector3d::UnitZ());
}

inline Joint::Joint(std::string name):
	dir_(1),
	params_(0),
	dof_(0),
	name_(name)
{
	constructJoint(Joint::Type::Fixed, Eigen::Vector3d::UnitZ());
}

inline void Joint::makeMimic(const std::string & name, double multiplier, double offset)
{
	isMimic_ = true;
	mimicName_ = name;
	mimicMultiplier_ = multiplier;
	mimicOffset_ = offset;
}


template<typename T>
inline sva::PTransform<T> Joint::pose(const std::vector<T>& q) const
{
	using namespace Eigen;
	using namespace sva;
	Matrix3<T> rot;
	switch(type_)
	{
		case Rev:
			// minus S because rotation is anti trigonometric
			return PTransform<T>(AngleAxis<T>(-q[0], S_.block<3, 1>(0, 0).cast<T>()).matrix());
		case Prism:
			return PTransform<T>(Vector3<T>(S_.block<3, 1>(3, 0).cast<T>()*q[0]));
		case Spherical:
			return PTransform<T>(Quaternion<T>(q[0], dir_*q[1], dir_*q[2], dir_*q[3]).inverse());
		case Planar:
			rot = sva::RotZ(q[0]);
			if(dir_ == 1.)
			{
				return PTransform<T>(rot, rot.transpose()*Vector3d(q[1], q[2], 0.));
			}
			else
			{
				return PTransform<T>(rot, rot.transpose()*Vector3d(q[1], q[2], 0.)).inv();
			}
		case Cylindrical:
			return PTransform<T>(AngleAxis<T>(-q[0], S_.col(0).head<3>().cast<T>()).matrix(),
													S_.col(1).tail<3>().cast<T>()*q[1]);
		case Free:
			rot = QuatToE(q);
			if(dir_ == 1.)
			{
				return PTransform<T>(rot,
					Vector3<T>(q[4], q[5], q[6]));
			}
			else
			{
				return PTransform<T>(rot,
					Vector3<T>(q[4], q[5], q[6])).inv();
			}
		case Fixed:
		default:
			return PTransform<T>::Identity();
	}
}


inline sva::MotionVecd Joint::motion(const std::vector<double>& alpha) const
{
	using namespace Eigen;
	using namespace sva;
	switch(type_)
	{
		case Rev:
			return MotionVecd((Vector6d() << S_.block<3, 1>(0, 0)*alpha[0],
																			 Vector3d::Zero()).finished());
		case Prism:
			return MotionVecd((Vector6d() << Vector3d::Zero(),
																			 S_.block<3, 1>(3, 0)*alpha[0]).finished());
		case Spherical:
			return MotionVecd(S_*Vector3d(alpha[0], alpha[1], alpha[2]));
		case Planar:
			return MotionVecd(S_*Vector3d(alpha[0], alpha[1], alpha[2]));
		case Cylindrical:
			return MotionVecd(S_*Vector2d(alpha[0], alpha[1]));
		case Free:
			return MotionVecd(S_*(Vector6d() << alpha[0], alpha[1], alpha[2],
								alpha[3], alpha[4], alpha[5]).finished());
		case Fixed:
		default:
			return MotionVecd(Vector6d::Zero());
	}
}


inline sva::MotionVecd Joint::tanAccel(const std::vector<double>& alphaD) const
{
	using namespace Eigen;
	using namespace sva;
	switch(type_)
	{
		case Rev:
			return MotionVecd((Vector6d() << S_.block<3, 1>(0, 0)*alphaD[0],
																			 Vector3d::Zero()).finished());
		case Prism:
			return MotionVecd((Vector6d() << Vector3d::Zero(),
																			 S_.block<3, 1>(3, 0)*alphaD[0]).finished());
		case Spherical:
			return MotionVecd(S_*Vector3d(alphaD[0], alphaD[1], alphaD[2]));
		case Planar:
			return MotionVecd(S_*Vector3d(alphaD[0], alphaD[1], alphaD[2]));
		case Cylindrical:
			return MotionVecd(S_*Vector2d(alphaD[0], alphaD[1]));
		case Free:
			return MotionVecd(S_*(Vector6d() << alphaD[0], alphaD[1], alphaD[2],
								alphaD[3], alphaD[4], alphaD[5]).finished());
		case Fixed:
		default:
			return MotionVecd(Vector6d::Zero());
	}
}


inline std::vector<double> Joint::zeroParam() const
{
	auto q = ZeroParam(type_);
	if(isMimic_)
	{
		for(auto & qi : q)
		{
			qi += mimicOffset_;
		}
	}
	return q;
}


inline std::vector<double> Joint::zeroDof() const
{
	return ZeroDof(type_);
}


inline sva::PTransformd Joint::sPose(const std::vector<double>& q) const
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


inline sva::MotionVecd Joint::sMotion(const std::vector<double>& alpha) const
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


inline sva::MotionVecd Joint::sTanAccel(const std::vector<double>& alphaD) const
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


inline std::vector<double> Joint::ZeroParam(Type type)
{
	switch(type)
	{
		case Rev:
		case Prism:
			return {0.};
		case Spherical:
			return {1., 0., 0., 0.};
		case Planar:
			return {0., 0., 0.};
		case Cylindrical:
			return {0., 0.};
		case Free:
			return {1., 0., 0., 0., 0., 0., 0.};
		case Fixed:
		default:
			return {};
	}
}


inline std::vector<double> Joint::ZeroDof(Type type)
{
	switch(type)
	{
		case Rev:
		case Prism:
			return {0.};
		case Spherical:
			return {0., 0., 0.};
		case Planar:
			return {0., 0., 0.};
		case Cylindrical:
			return {0., 0.};
		case Free:
			return {0., 0., 0., 0., 0., 0.};
		case Fixed:
		default:
			return {};
	}
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
		case Planar:
			S_ = Matrix<double, 6, 3>::Zero();
			S_.block<3, 3>(2, 0).setIdentity();
			S_ *= dir_;
			params_ = 3;
			dof_ = 3;
			break;
		case Cylindrical:
			S_ = Matrix<double, 6, 2>::Zero();
			S_.col(0).head<3>() = a;
			S_.col(1).tail<3>() = a;
			S_ *= dir_;
			params_ = 2;
			dof_ = 2;
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


template<typename T>
inline Eigen::Matrix3<T> QuatToE(const std::vector<T>& q)
{
	using namespace Eigen;
	T p0 = q[0];
	T p1 = q[1];
	T p2 = q[2];
	T p3 = q[3];

	T p0p1 = p0*p1;
	T p0p2 = p0*p2;
	T p0p3 = p0*p3;

	T p1p2 = p1*p2;
	T p1p3 = p1*p3;

	T p2p3 = p2*p3;

	T p0s = std::pow(p0, 2);
	T p1s = std::pow(p1, 2);
	T p2s = std::pow(p2, 2);
	T p3s = std::pow(p3, 2);

	return 2.*(Matrix3<T>() << p0s + p1s - 0.5, p1p2 + p0p3, p1p3 - p0p2,
														p1p2 - p0p3, p0s + p2s - 0.5, p2p3 + p0p1,
														p1p3 + p0p2, p2p3 - p0p1, p0s + p3s - 0.5).finished();
}

} // namespace rbd
