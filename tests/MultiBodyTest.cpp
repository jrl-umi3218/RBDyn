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
#define BOOST_TEST_MODULE MultiBodyTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg>

// RBDyn
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyGraph.h"


BOOST_AUTO_TEST_CASE(MultiBodyGraphTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg1;

	// test addBody
	Body b1(RBInertia(), 0, "b1");
	Body b2(RBInertia(), 1, "b2");
	Body b3(RBInertia(), 2, "b3");
	Body b4(RBInertia(), 3, "b4");
	BOOST_CHECK_NO_THROW(mbg1.addBody(b1));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b2));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b3));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b4));

	// id already exist
	BOOST_CHECK_THROW(mbg1.addBody(Body(RBInertia(), 0, "b3")), std::domain_error);

	// must be 4 nodes
	BOOST_CHECK_EQUAL(mbg1.nrNodes(), 4);

	// test nodeById
	std::shared_ptr<MultiBodyGraph::Node> node1, node2;
	BOOST_CHECK_NO_THROW(node1 = mbg1.nodeById(0));
	BOOST_CHECK_NO_THROW(node2 = mbg1.nodeById(1));
	BOOST_CHECK_EQUAL(node1->body, b1);
	BOOST_CHECK_EQUAL(node2->body, b2);

	// check non-existant id
	BOOST_CHECK_THROW(mbg1.nodeById(10), std::out_of_range);


	// test addJoint
	Joint j1(Joint::RevX, true, 0, "j1");
	Joint j2(Joint::RevX, true, 1, "j2");
	Joint j3(Joint::RevX, true, 2, "j3");
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j1));
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j2));
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j3));

	// id already exist
	BOOST_CHECK_THROW(mbg1.addJoint(Joint(Joint::RevX, true, 0, "j4")), std::domain_error);

	// must be 3 joints
	BOOST_CHECK_EQUAL(mbg1.nrJoints(), 3);

	// test jointById
	std::shared_ptr<Joint> joint1, joint2, joint3;
	BOOST_CHECK_NO_THROW(joint1 = mbg1.jointById(0));
	BOOST_CHECK_NO_THROW(joint2 = mbg1.jointById(1));
	BOOST_CHECK_NO_THROW(joint3 = mbg1.jointById(2));

	BOOST_CHECK_EQUAL(*joint1, j1);
	BOOST_CHECK_EQUAL(*joint2, j2);
	BOOST_CHECK_EQUAL(*joint3, j3);

	// check non-existant id
	BOOST_CHECK_THROW(mbg1.jointById(10), std::out_of_range);

	// test linkBody
	//        b2(1)
	//   j1(0)  /  j2(1)     j3(2)
	//       b1(0) ---- b3(2) ---- b4(3)

	BOOST_CHECK_NO_THROW(mbg1.linkBodies(0, PTransform::Identity(),
		1, PTransform::Identity(), 0));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies(0, PTransform::Identity(),
		2, PTransform::Identity(), 1));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies(2, PTransform::Identity(),
		3, PTransform::Identity(), 2));

	// check non-existant body 1
	BOOST_CHECK_THROW(mbg1.linkBodies(10, PTransform::Identity(),
		3, PTransform::Identity(), 2), std::out_of_range);
	// check non-existant body 2
	BOOST_CHECK_THROW(mbg1.linkBodies(2, PTransform::Identity(),
		10, PTransform::Identity(), 2), std::out_of_range);
	// check non-existant joint
	BOOST_CHECK_THROW(mbg1.linkBodies(2, PTransform::Identity(),
		3, PTransform::Identity(), 10), std::out_of_range);
}



void checkMultiBodyEq(const rbd::MultiBody& mb, std::vector<rbd::Body> bodies,
	std::vector<rbd::Joint> joints, std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent,
	std::vector<sva::PTransform> Xt)
{
	// bodies
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.bodies().begin(), mb.bodies().end(),
																bodies.begin(), bodies.end());
	// joints
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.joints().begin(), mb.joints().end(),
																joints.begin(), joints.end());
	// pred
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.predecessors().begin(), mb.predecessors().end(),
																pred.begin(), pred.end());
	// succ
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.successors().begin(), mb.successors().end(),
																succ.begin(), succ.end());
	// parent
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.parents().begin(), mb.parents().end(),
																parent.begin(), parent.end());

	// Xt
	BOOST_CHECK_EQUAL_COLLECTIONS(mb.transforms().begin(),
		mb.transforms().end(),
		Xt.begin(), Xt.end());

	// nrBodies
	BOOST_CHECK_EQUAL(mb.nrBodies(), bodies.size());
	// nrJoints
	BOOST_CHECK_EQUAL(mb.nrJoints(), bodies.size());

	int params = 0, dof = 0;
	for(std::size_t i = 0; i < joints.size(); ++i)
	{
		params += joints[i].params();
		dof += joints[i].dof();
	}

	BOOST_CHECK_EQUAL(params, mb.nrParams());
	BOOST_CHECK_EQUAL(dof, mb.nrDof());
}



BOOST_AUTO_TEST_CASE(MultiBodyTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	//                   b2(1)
	//             j1(0)  /  j2(1)     j3(2)
	// root --j0(-1)-- b1(0) ---- b3(2) ---- b4(3)

	RBInertia r;
	std::vector<Body> bodies = {Body(r, 0, "b1"),
															Body(r, 1, "b2"),
															Body(r, 6, "b3"),
															Body(r, 5, "b4")};

	std::vector<Joint> joints = {Joint(Joint::RevX, true, -1, "j0"),
															 Joint(Joint::RevX, true, 0, "j1"),
															 Joint(Joint::RevX, true, 11, "j2"),
															 Joint(Joint::RevX, true, 4, "j3")};

	std::vector<int> pred = {-1, 0, 0, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 0, 2};
	Vector3d tmp = Vector3d::Random();

	PTransform I = PTransform::Identity();
	std::vector<PTransform> Xt = {I, I, PTransform(tmp), I};

	MultiBody mb(bodies, joints, pred, succ, parent, Xt);

	// check MultiBody equality
	checkMultiBodyEq(mb, bodies, joints, pred, succ, parent, Xt);

	// Id2Index
	// bodyIndexById
	BOOST_CHECK_EQUAL(mb.bodyIndexById(0), 0);
	BOOST_CHECK_EQUAL(mb.bodyIndexById(1), 1);
	BOOST_CHECK_EQUAL(mb.bodyIndexById(6), 2);
	BOOST_CHECK_EQUAL(mb.bodyIndexById(5), 3);

	// jointIndexById
	BOOST_CHECK_EQUAL(mb.jointIndexById(-1), 0);
	BOOST_CHECK_EQUAL(mb.jointIndexById(0), 1);
	BOOST_CHECK_EQUAL(mb.jointIndexById(11), 2);
	BOOST_CHECK_EQUAL(mb.jointIndexById(4), 3);

	// safe accessors
	// body
	BOOST_CHECK_NO_THROW(mb.sBody(0));
	BOOST_CHECK_THROW(mb.sBody(10), std::out_of_range);

	// joint
	BOOST_CHECK_NO_THROW(mb.sJoint(0));
	BOOST_CHECK_THROW(mb.sJoint(10), std::out_of_range);

	// pred
	BOOST_CHECK_NO_THROW(mb.sPredecessor(0));
	BOOST_CHECK_THROW(mb.sPredecessor(10), std::out_of_range);

	// succ
	BOOST_CHECK_NO_THROW(mb.sSuccessor(0));
	BOOST_CHECK_THROW(mb.sSuccessor(10), std::out_of_range);

	// parent
	BOOST_CHECK_NO_THROW(mb.sParent(0));
	BOOST_CHECK_THROW(mb.sParent(10), std::out_of_range);

	// transformFrom
	BOOST_CHECK_NO_THROW(mb.sTransform(0));
	BOOST_CHECK_THROW(mb.sTransform(10), std::out_of_range);

	// bodyIndexById
	BOOST_CHECK_NO_THROW(mb.sBodyIndexById(0));
	BOOST_CHECK_THROW(mb.sBodyIndexById(10), std::out_of_range);

	// jointIndexById
	BOOST_CHECK_NO_THROW(mb.sJointIndexById(0));
	BOOST_CHECK_THROW(mb.sJointIndexById(10), std::out_of_range);
}



BOOST_AUTO_TEST_CASE(MakeMultiBodyTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	MultiBodyGraph mbg1;

	Body b1(RBInertia(), 0, "b1");
	Body b2(RBInertia(), 1, "b2");
	Body b3(RBInertia(), 2, "b3");
	Body b4(RBInertia(), 3, "b4");
	mbg1.addBody(b1);
	mbg1.addBody(b2);
	mbg1.addBody(b3);
	mbg1.addBody(b4);

	Joint j1(Joint::RevX, true, 0, "j1");
	Joint j2(Joint::RevX, true, 1, "j2");
	Joint j3(Joint::RevX, true, 2, "j3");
	mbg1.addJoint(j1);
	mbg1.addJoint(j2);
	mbg1.addJoint(j3);

	//        b2(1)
	//   j1(0)  /  j2(1)     j3(2)
	//       b1(0) ---- b3(2) ---- b4(3)
	mbg1.linkBodies(0, PTransform::Identity(), 1, PTransform::Identity(), 0);
	mbg1.linkBodies(0, PTransform::Identity(), 2, PTransform::Identity(), 1);
	mbg1.linkBodies(2, PTransform::Identity(), 3, PTransform::Identity(), 2);


	//                b2(1)
	//          j1(0)  /  j2(1)     j3(2)
	// root -fixed- b1(0) ---- b3(2) ---- b4(3)

	BOOST_CHECK_NO_THROW(mbg1.makeMultiBody(0, true));
	BOOST_CHECK_THROW(mbg1.makeMultiBody(10, true), std::out_of_range);

	MultiBody mb1 = mbg1.makeMultiBody(0, true);


	std::vector<Body> bodies = {b1, b2, b3, b4};

	std::vector<Joint> joints = {Joint(Joint::Fixed, true, -1, "Root"),
		j1, j2, j3};

	std::vector<int> pred = {-1, 0, 0, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 0, 2};

	PTransform I = PTransform::Identity();
	std::vector<PTransform> Xt = {I, I, I, I};

	// check MultiBody equality
	checkMultiBodyEq(mb1, bodies, joints, pred, succ, parent, Xt);

	// check bodyIndexById
	BOOST_CHECK_EQUAL(mb1.bodyIndexById(0), 0);
	BOOST_CHECK_EQUAL(mb1.bodyIndexById(1), 1);
	BOOST_CHECK_EQUAL(mb1.bodyIndexById(2), 2);
	BOOST_CHECK_EQUAL(mb1.bodyIndexById(3), 3);

	// check jointIndexById
	BOOST_CHECK_EQUAL(mb1.jointIndexById(-1), 0);
	BOOST_CHECK_EQUAL(mb1.jointIndexById(0), 1);
	BOOST_CHECK_EQUAL(mb1.jointIndexById(1), 2);
	BOOST_CHECK_EQUAL(mb1.jointIndexById(2), 3);



	//                b2(1)
	//          j1(0)  /  j2(1)     j3(2)
	// root -free- b1(0) ---- b3(2) ---- b4(3)

	MultiBody mb2 = mbg1.makeMultiBody(0, false);
	joints = {Joint(Joint::Free, true, -1, "Root"), j1, j2, j3};

	// check MultiBody equality
	checkMultiBodyEq(mb2, bodies, joints, pred, succ, parent, Xt);



	//                     j1(0)     j2(1)      j3(2)
	// root -fixed-  b2(1) ---- b1(0) ---- b3(2) ---- b4(3)

	MultiBody mb3 = mbg1.makeMultiBody(1, true);

	bodies = {b2, b1, b3, b4};

	j1.forward(false);
	joints = {Joint(Joint::Fixed, true, -1, "Root"), j1, j2, j3};

	pred = {-1, 0, 1, 2};
	succ = {0, 1, 2, 3};
	parent = {-1, 0, 1, 2};

	Xt = {I, I, I, I};

	// check joint j1 direction
	// check bodyIndexById
	BOOST_CHECK_EQUAL(mb3.joint(1).forward(), false);

	// check MultiBody equality
	checkMultiBodyEq(mb3, bodies, joints, pred, succ, parent, Xt);

	// check bodyIndexById
	BOOST_CHECK_EQUAL(mb3.bodyIndexById(0), 1);
	BOOST_CHECK_EQUAL(mb3.bodyIndexById(1), 0);
	BOOST_CHECK_EQUAL(mb3.bodyIndexById(2), 2);
	BOOST_CHECK_EQUAL(mb3.bodyIndexById(3), 3);

	// check jointIndexById
	BOOST_CHECK_EQUAL(mb3.jointIndexById(-1), 0);
	BOOST_CHECK_EQUAL(mb3.jointIndexById(0), 1);
	BOOST_CHECK_EQUAL(mb3.jointIndexById(1), 2);
	BOOST_CHECK_EQUAL(mb3.jointIndexById(2), 3);



	// check transform
	//                                  tx(1)-tz(-1) --- B3
	// root -fixed- B1 -tx(1)-ty(-1)- B2 /
	//                                   \ rx(90)-I ---- B4

	MultiBodyGraph mbg2;

	mbg2.addBody(b1);
	mbg2.addBody(b2);
	mbg2.addBody(b3);
	mbg2.addBody(b4);

	mbg2.addJoint(j1);
	mbg2.addJoint(j2);
	mbg2.addJoint(j3);

	mbg2.linkBodies(0, PTransform(Vector3d(Vector3d::UnitX())),
									1, PTransform(Vector3d(-Vector3d::UnitY())), 0);
	mbg2.linkBodies(1, PTransform(Vector3d(Vector3d::UnitX())),
									2, PTransform(Vector3d(-Vector3d::UnitZ())), 1);
	mbg2.linkBodies(1, PTransform(Matrix3d(sva::RotX(constants::pi<double>()/2.))),
									3, PTransform::Identity(), 2);

	MultiBody mb4 = mbg2.makeMultiBody(0, true);


	bodies = {b1, b2, b3, b4};

	joints = {Joint(Joint::Fixed, true, -1, "Root"), j1, j2, j3};

	pred = {-1, 0, 1, 1};
	succ = {0, 1, 2, 3};
	parent = {-1, 0, 1, 1};

	Xt = {I,
				PTransform(Vector3d(1., 0., 0.)),
				PTransform(Vector3d(1., 1., 0.)),
				PTransform(RotX(constants::pi<double>()/2.), Vector3d(0., 1., 0.))};

	// check MultiBody equality
	checkMultiBodyEq(mb4, bodies, joints, pred, succ, parent, Xt);
}
