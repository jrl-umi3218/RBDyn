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
#include <string>

// SpaceVecAlg
#include <SpaceVecAlg>

namespace rbd
{

class Joint
{
public:
	enum Type {RevX, RevY, RevZ, PrismX, PrismY, PrismZ, Spherical, Free, Fixed};

public:
	Joint(Type type, int id, std::string name);

	Type type() const
	{
		return type_;
	}

	int params() const
	{
		return params_;
	}

	int dof() const
	{
		return dof_;
	}

	int id() const
	{
		return id_;
	}

	const std::string& name() const
	{
		return name_;
	}

	const Eigen::MatrixXd& motionSubspace() const
	{
		return S_;
	}

	sva::PTransform pose(const std::vector<double>& q) const;
	sva::MotionVec motion(const std::vector<double>& alpha) const;

private:
	Type type_;
	Eigen::MatrixXd S_;

	int params_;
	int dof_;

	int id_;
	std::string name_;
};


inline Joint::Joint(Type type, int id, std::string name):
	type_(type),
	id_(id),
	name_(name)
{
	using namespace Eigen;
	switch(type_)
	{
		case RevX:
			S_ = Vector6d::UnitX();
			params_ = 1;
			dof_ = 1;
			break;
		case RevY:
			S_ = Vector6d::UnitY();
			params_ = 1;
			dof_ = 1;
			break;
		case RevZ:
			S_ = Vector6d::UnitZ();
			params_ = 1;
			dof_ = 1;
			break;
		case PrismX:
			S_ = (Vector6d() << 0., 0., 0., 1., 0., 0.).finished();
			params_ = 1;
			dof_ = 1;
			break;
		case PrismY:
			S_ = (Vector6d() << 0., 0., 0., 0., 1., 0.).finished();
			params_ = 1;
			dof_ = 1;
			break;
		case PrismZ:
			S_ = (Vector6d() << 0., 0., 0., 0., 0., 1.).finished();
			params_ = 1;
			dof_ = 1;
			break;
		case Spherical:
			S_ = Matrix<double, 6, 3>::Zero();
			S_.block<3, 3>(0, 0).setIdentity();
			params_ = 4;
			dof_ = 3;
			break;
		case Free:
			S_ = Matrix6d::Identity();
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

inline sva::PTransform Joint::pose(const std::vector<double>& q) const
{
	using namespace Eigen;
	using namespace sva;
	Matrix3d rot;
	switch(type_)
	{
		case RevX:
			return PTransform(RotX(q[0]));
		case RevY:
			return PTransform(RotY(q[0]));
		case RevZ:
			return PTransform(RotZ(q[0]));
		case PrismX:
			return PTransform((Vector3d() << q[0], 0., 0.).finished());
		case PrismY:
			return PTransform((Vector3d() << 0., q[0], 0.).finished());
		case PrismZ:
			return PTransform((Vector3d() << 0., 0., q[0]).finished());
		case Spherical:
			return PTransform(Quaterniond(q[0], q[1], q[2], q[3]));
		case Free:
			// must be in successor frame coordinate
			rot = Quaterniond(q[0], q[1], q[2], q[3]).matrix();
			return PTransform(rot.transpose(), rot*Vector3d(q[4], q[5], q[6]));
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
		case RevX:
			return MotionVec((Vector6d() << alpha[0], 0., 0., 0., 0., 0.).finished());
		case RevY:
			return MotionVec((Vector6d() << 0., alpha[0], 0., 0., 0., 0.).finished());
		case RevZ:
			return MotionVec((Vector6d() << 0., 0., alpha[0], 0., 0., 0.).finished());
		case PrismX:
			return MotionVec((Vector6d() << 0., 0., 0., alpha[0], 0., 0.).finished());
		case PrismY:
			return MotionVec((Vector6d() << 0., 0., 0., 0., alpha[0], 0.).finished());
		case PrismZ:
			return MotionVec((Vector6d() << 0., 0., 0., 0., 0., alpha[0]).finished());
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

} // namespace rbd
