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
		* @param name Body name, must be unique in a multibody.
		*/
	Body(const sva::RBInertiad& rbInertia, std::string name):
		inertia_(rbInertia),
		name_(name)
	{}

	/**
		* @param mass Body mass.
		* @param com Body center of mass.
		* @param inertia Body inertia matrix at body origin.
		* @param name Body name, must be unique in a multibody.
		*/
	Body(double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia,
		std::string name):
		inertia_(mass, mass*com, inertia),
		name_(name)
	{}

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
		return name_ == b.name_;
	}

	bool operator!=(const Body& b) const
	{
		return name_ != b.name_;
	}

private:
	sva::RBInertiad inertia_;
	std::string name_;

};

inline std::ostream& operator<<(std::ostream& out, const Body& b)
{
	out << "Body: " << b.name();
	return out;
}

} // namespace rbd
