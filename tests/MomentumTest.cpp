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
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MomentumTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// RBDyn
#include "CoM.h"
#include "EulerIntegration.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "Momentum.h"
#include "MultiBody.h"
#include "MultiBodyConfig.h"

// arm
#include "XYZSarm.h"

const double TOL = 1e-6;

BOOST_AUTO_TEST_CASE(centroidalMomentum)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm();

	VectorXd q(mb.nrParams());
	VectorXd alpha(mb.nrDof());
	CentroidalMomentumMatrix cmm(mb);

	{
		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);
		rbd::paramToVector(mbc.alpha, alpha);

		Vector3d com = rbd::computeCoM(mb, mbc);
		ForceVecd momentum = rbd::computeCentroidalMomentum(mb, mbc, com);

		ForceVecd momentumM(cmm.matrix(mb, mbc, com)*alpha);

		BOOST_CHECK_EQUAL(momentum.vector().norm(), 0.);
		BOOST_CHECK_EQUAL(momentumM.vector().norm(), 0.);
	}

	for(int i = 0; i < 100; ++i)
	{
		q.setRandom();
		q.segment<4>(mb.jointPosInParam(mb.jointIndexById(3))).normalize();
		alpha.setRandom();
		rbd::vectorToParam(q, mbc.q);
		rbd::vectorToParam(alpha, mbc.alpha);

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		Vector3d com = rbd::computeCoM(mb, mbc);
		ForceVecd momentum = rbd::computeCentroidalMomentum(mb, mbc, com);

		ForceVecd momentumM(cmm.matrix(mb, mbc, com)*alpha);

		BOOST_CHECK_SMALL((momentum - momentumM).vector().norm(), TOL);
	}
}



BOOST_AUTO_TEST_CASE(centroidalMomentumDot)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace cst = boost::math::constants;

	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	std::tie(mb, mbc, mbg) = makeXYZSarm();

	VectorXd q(mb.nrParams());
	VectorXd alpha(mb.nrDof());
	VectorXd alphaD(mb.nrDof());

	for(int i = 0; i < 10; ++i)
	{
		q.setRandom();
		q.segment<4>(mb.jointPosInParam(mb.jointIndexById(3))).normalize();
		alpha.setRandom();
		alphaD.setRandom();
		rbd::vectorToParam(q, mbc.q);
		rbd::vectorToParam(alpha, mbc.alpha);
		rbd::vectorToParam(alphaD, mbc.alphaD);

		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);
		rbd::forwardAcceleration(mb, mbc);


		for(int j = 0; j < 10; ++j)
		{
			Vector3d oldCom = rbd::computeCoM(mb, mbc);
			ForceVecd oldMomentum = rbd::computeCentroidalMomentum(mb, mbc, oldCom);
			Vector3d oldComVel = rbd::computeCoMVelocity(mb, mbc);
			ForceVecd momentumDot = rbd::computeCentroidalMomentumDot(mb, mbc,
				oldCom, oldComVel);

			rbd::eulerIntegration(mb, mbc, 1e-8);

			rbd::forwardKinematics(mb, mbc);
			rbd::forwardVelocity(mb, mbc);
			rbd::forwardAcceleration(mb, mbc);

			Vector3d newCom = rbd::computeCoM(mb, mbc);
			ForceVecd newMomentum = rbd::computeCentroidalMomentum(mb, mbc, newCom);
			ForceVecd momentumDotDiff = (newMomentum - oldMomentum)*(1./1e-8);


			BOOST_CHECK_SMALL((momentumDot - momentumDotDiff).vector().norm(), TOL);
		}
	}
}

