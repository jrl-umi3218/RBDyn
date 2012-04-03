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
	bodyPosW(mb.nrBodies()),
	bodyVelW(mb.nrBodies())
{
	for(std::size_t i = 0; i < q.size(); ++i)
	{
		q[i].resize(mb.joint(i).params());
		alpha[i].resize(mb.joint(i).dof());
	}
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



void checkMatchBodyVel(const MultiBody& mb, const MultiBodyConfig& mbc)
{
	if(mbc.bodyVelW.size() != mb.nrBodies())
	{
		std::ostringstream str;
		str << "bodyVelW size mismatch: expected size "
				<< mb.nrBodies() << " gived " << mbc.bodyVelW.size();
		throw std::domain_error(str.str());
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
			str << "Bad number of generalized position variabel for Joint "
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
			str << "Bad number of generalized velocity variabel for Joint "
					<< mb.joint(i) << " at position " << i << ": expected size "
					<< mb.joint(i).dof() << " gived " << mbc.alpha[i].size();
			throw std::domain_error(str.str());
		}
	}
}

} // namespace rbd
