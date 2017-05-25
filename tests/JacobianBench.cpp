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

// includes
// std
#include <iostream>

// boost
#define BOOST_TEST_MODULE JacobianBench
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/timer/timer.hpp>
// The inclusion of boost chrono was commented in timer.hpp for boost >= 1.60.
// Because of this, the auto-link feature does not incude the chrono library
// anymore, what causes a link error. 
// (see also https://svn.boost.org/trac/boost/ticket/11862)
// We add manually the line.
// Possible alternative: include only for specific version of boost and 
// auto-link capable compiler
#include <boost/chrono/chrono.hpp>

// RBDyn
#include "RBDyn/CoM.h"
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include "RBDyn/Jacobian.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyConfig.h"
#include "RBDyn/MultiBodyGraph.h"

// Arm
#include "Tree30Dof.h"

BOOST_AUTO_TEST_CASE(Jacobian)
{
	const std::size_t nrIteration = 100000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::Jacobian jac(mb, "LARM6");

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "Jacobian::jacobian" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobian(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(BodyJacobian)
{
	const std::size_t nrIteration = 100000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::Jacobian jac(mb, "LARM6");

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "Jacobian::bodyJacobian" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.bodyJacobian(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(VectorBodyJacobian)
{
	const std::size_t nrIteration = 100000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::Jacobian jac(mb, "LARM6");

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "Jacobian::vectorBodyJacobian" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.vectorBodyJacobian(mb, mbc, Eigen::Vector3d(1., 0., 0.));
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(JacobianDot)
{
	const std::size_t nrIteration = 100000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::Jacobian jac(mb, "LARM6");

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "Jacobian::jacobianDot" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobianDot(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(BodyJacobianDot)
{
	const std::size_t nrIteration = 100000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::Jacobian jac(mb, "LARM6");

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "Jacobian::bodyJacobianDot" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.bodyJacobianDot(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(CoMJacobianDummy_jacobian)
{
	const std::size_t nrIteration = 10000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::CoMJacobianDummy jac(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "CoMJacobianDummy::jacobian" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobian(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(CoMJacobianDummy_jacobianDot)
{
	const std::size_t nrIteration = 10000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::CoMJacobianDummy jac(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "CoMJacobianDummy::jacobianDot" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobianDot(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(CoMJacobian_jacobian)
{
	const std::size_t nrIteration = 10000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::CoMJacobian jac(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "CoMJacobian::jacobian" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobian(mb, mbc);
		}
	}
	std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE(CoMJacobian_jacobianDot)
{
	const std::size_t nrIteration = 10000;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeTree30Dof(false);

	rbd::CoMJacobian jac(mb);

	rbd::forwardKinematics(mb, mbc);
	rbd::forwardVelocity(mb, mbc);

	std::cout << "CoMJacobian::jacobianDot" << std::endl;
	{
		boost::timer::auto_cpu_timer t;
		for(std::size_t i = 0; i < nrIteration; ++i)
		{
			jac.jacobianDot(mb, mbc);
		}
	}
	std::cout << std::endl;
}
