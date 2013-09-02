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
#include <SpaceVecAlg/SpaceVecAlg>

namespace rbd
{

/** Body representation.
	* Hold body id, name and spatial rigid body inertia of one body.
	*/
class Body
{
public:
	Body()
	{}

	/**
		* @param rbInertia Body spatial rigid body inertia.
		* @param id Body id, must be unique in a multibody.
		* @param name Body name.
		*/
	Body(const sva::RBInertiad& rbInertia, int id, std::string name):
		inertia_(rbInertia),
		id_(id),
		name_(name)
	{}

	/**
		* @param mass Body mass.
		* @param com Body center of mass.
		* @param inertia Body inertia matrix at body origin.
		* @param id Body id, must be unique in a multibody.
		* @param name Body name.
		*/
	Body(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia,
		int id, std::string name):
		inertia_(mass, mass*com, inertia),
		id_(id),
		name_(name)
	{}

	/// @return Body id.
	int id() const
	{
		return id_;
	}

	/// @return Body name.
	const std::string& name() const
	{
		return name_;
	}

	/// @return Body spatial rigid body inertia.
	const sva::RBInertiad& inertia() const
	{
		return inertia_;
	}

	bool operator==(const Body& b) const
	{
		return id_ == b.id_ && name_ == b.name_;
	}

	bool operator!=(const Body& b) const
	{
		return id_ != b.id_ || name_ != b.name_;
	}

private:
	sva::RBInertiad inertia_;

	int id_;
	std::string name_;
};

inline std::ostream& operator<<(std::ostream& out, const Body& b)
{
	out << "Body: " << b.id() << ", " << b.name();
	return out;
}

} // namespace rbd
