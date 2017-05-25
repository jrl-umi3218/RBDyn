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
#define BOOST_TEST_MODULE MultiBodyTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Body.h"
#include "RBDyn/FK.h"
#include "RBDyn/Joint.h"
#include "RBDyn/MultiBody.h"
#include "RBDyn/MultiBodyGraph.h"
#include "RBDyn/MultiBodyConfig.h"

// arm
#include "XYZSarm.h"


BOOST_AUTO_TEST_CASE(MultiBodyGraphTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg1;

	// test addBody
	Body b1(RBInertiad(), "b1");
	Body b2(RBInertiad(), "b2");
	Body b3(RBInertiad(), "b3");
	Body b4(RBInertiad(), "b4");
	BOOST_CHECK_NO_THROW(mbg1.addBody(b1));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b2));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b3));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b4));

	// name already exists
	BOOST_CHECK_THROW(mbg1.addBody(Body(RBInertiad(), "b3")), std::domain_error);

	// must be 4 nodes
	BOOST_CHECK_EQUAL(mbg1.nrNodes(), 4);

	// test nodeByName
	std::shared_ptr<MultiBodyGraph::Node> node1, node2;
	BOOST_CHECK_NO_THROW(node1 = mbg1.nodeByName("b1"));
	BOOST_CHECK_NO_THROW(node2 = mbg1.nodeByName("b2"));
	BOOST_CHECK_EQUAL(node1->body, b1);
	BOOST_CHECK_EQUAL(node2->body, b2);

	// test addJoint
	Joint j1(Joint::RevX, true, "j1");
	Joint j2(Joint::RevX, true, "j2");
	Joint j3(Joint::RevX, true, "j3");
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j1));
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j2));
	BOOST_CHECK_NO_THROW(mbg1.addJoint(j3));

	// name already exists
	BOOST_CHECK_THROW(mbg1.addJoint(Joint(Joint::RevX, true, "j3")), std::domain_error);

	// must be 3 joints
	BOOST_CHECK_EQUAL(mbg1.nrJoints(), 3);

	// test jointByName
	std::shared_ptr<Joint> joint1, joint2, joint3;
	BOOST_CHECK_NO_THROW(joint1 = mbg1.jointByName("j1"));
	BOOST_CHECK_NO_THROW(joint2 = mbg1.jointByName("j2"));
	BOOST_CHECK_NO_THROW(joint3 = mbg1.jointByName("j3"));

	BOOST_CHECK_EQUAL(*joint1, j1);
	BOOST_CHECK_EQUAL(*joint2, j2);
	BOOST_CHECK_EQUAL(*joint3, j3);

	// check non-existant name
	BOOST_CHECK_THROW(mbg1.jointByName("j10"), std::out_of_range);

	// test linkBody
	//        b2
	//   j1  /   j2      j3
	//       b1 ---- b3 ---- b4

	BOOST_CHECK_NO_THROW(mbg1.linkBodies("b1", PTransformd::Identity(),
		"b2", PTransformd::Identity(), "j1"));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies("b1", PTransformd::Identity(),
		"b3", PTransformd::Identity(), "j2"));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies("b3", PTransformd::Identity(),
		"b4", PTransformd::Identity(), "j3"));

	// check non-existant body 1
	BOOST_CHECK_THROW(mbg1.linkBodies("b10", PTransformd::Identity(),
		"b4", PTransformd::Identity(), "j3"), std::out_of_range);
	// check non-existant body 2
	BOOST_CHECK_THROW(mbg1.linkBodies("b3", PTransformd::Identity(),
		"b10", PTransformd::Identity(), "j3"), std::out_of_range);
	// check non-existant joint
	BOOST_CHECK_THROW(mbg1.linkBodies("b3", PTransformd::Identity(),
		"b4", PTransformd::Identity(), "j10"), std::out_of_range);
}



void checkMultiBodyNames(const rbd::MultiBody& mb,
											 const std::vector<std::string>& bodies,
											 const std::vector<std::string>& joints)
{
	for(const rbd::Body& b: mb.bodies())
	{
		BOOST_CHECK(std::find(bodies.begin(), bodies.end(), b.name()) != bodies.end());
	}
	for(const rbd::Joint& j: mb.joints())
	{
		BOOST_CHECK(std::find(joints.begin(), joints.end(), j.name()) != joints.end());
	}
}


BOOST_AUTO_TEST_CASE(MultiBodyGraphRmTest)
{
	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbgBack;
	std::tie(mb, mbc, mbgBack) = makeXYZSarm();

	// test copy constructor
	rbd::MultiBodyGraph mbg(mbgBack);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), mbgBack.nrJoints());
	BOOST_CHECK_EQUAL(mbg.nrNodes(), mbgBack.nrNodes());



	BOOST_CHECK_EQUAL(mbg.nrJoints(), 4);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 5);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 5);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 5);

	mbg.removeJoint("b0", "j3");

	mb = mbg.makeMultiBody("b0", true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 4);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 4);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 4);
	checkMultiBodyNames(mb, {"b0", "b1", "b2", "b3"}, {"Root", "j0", "j1", "j2"});


	// test operator=
	mbg = mbgBack;
	BOOST_CHECK_EQUAL(mbg.nrJoints(), mbgBack.nrJoints());
	BOOST_CHECK_EQUAL(mbg.nrNodes(), mbgBack.nrNodes());

	mbg.removeJoint("b0", "j0");
	mb = mbg.makeMultiBody("b0", true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 0);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 1);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 1);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 1);

	BOOST_CHECK_EQUAL(mb.body(0).name(), "b0");
	BOOST_CHECK_EQUAL(mb.joint(0).name(), "Root");


	mbg = mbgBack;
	BOOST_CHECK_EQUAL(mbg.nrJoints(), mbgBack.nrJoints());
	BOOST_CHECK_EQUAL(mbg.nrNodes(), mbgBack.nrNodes());

	mbg.removeJoints("b0", std::vector<std::string>({"j3", "j2"}));
	mb = mbg.makeMultiBody("b0", true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 2);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 3);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 3);
	checkMultiBodyNames(mb, {"b0", "b1", "b2"}, {"Root", "j0", "j1"});


	mbg = mbgBack;
	BOOST_CHECK_EQUAL(mbg.nrJoints(), mbgBack.nrJoints());
	BOOST_CHECK_EQUAL(mbg.nrNodes(), mbgBack.nrNodes());

	mbg.removeJoint("b0", "j1");
	mb = mbg.makeMultiBody("b0", true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 2);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 3);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 3);
	checkMultiBodyNames(mb, {"b0", "b1", "b4"}, {"Root", "j0", "j3"});


	mbg = mbgBack;
	BOOST_CHECK_EQUAL(mbg.nrJoints(), mbgBack.nrJoints());
	BOOST_CHECK_EQUAL(mbg.nrNodes(), mbgBack.nrNodes());

	mbg.removeJoints("b0", std::vector<std::string>({"j1", "j2"}));
	mb = mbg.makeMultiBody("b0", true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 2);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 3);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 3);
	checkMultiBodyNames(mb, {"b0", "b1", "b4"}, {"Root", "j0", "j3"});
}



void checkMultiBodyEq(const rbd::MultiBody& mb, std::vector<rbd::Body> bodies,
	std::vector<rbd::Joint> joints, std::vector<int> pred, std::vector<int> succ,
	std::vector<int> parent,
	std::vector<sva::PTransformd> Xt)
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
	for(int i = 0; i < static_cast<int>(joints.size()); ++i)
	{
		BOOST_CHECK_EQUAL(mb.jointPosInParam(i), params);
		BOOST_CHECK_EQUAL(mb.jointsPosInParam()[i], params);
		BOOST_CHECK_EQUAL(mb.sJointPosInParam(i), params);

		BOOST_CHECK_EQUAL(mb.jointPosInDof(i), dof);
		BOOST_CHECK_EQUAL(mb.jointsPosInDof()[i], dof);
		BOOST_CHECK_EQUAL(mb.sJointPosInDof(i), dof);

		params += joints[i].params();
		dof += joints[i].dof();
	}

	BOOST_CHECK_EQUAL(params, mb.nrParams());
	BOOST_CHECK_EQUAL(dof, mb.nrDof());

	BOOST_CHECK_THROW(mb.sJointPosInParam(mb.nrJoints()), std::out_of_range);
	BOOST_CHECK_THROW(mb.sJointPosInDof(mb.nrJoints()), std::out_of_range);
}



BOOST_AUTO_TEST_CASE(MultiBodyTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	//              b2
	//          j1  /  j2      j3
	// root --j0-- b1 ---- b6 ---- b5

	RBInertiad r(1., Vector3d::Random(), Matrix3d::Random());
	std::vector<Body> bodies = {Body(r, "b1"),
					Body(r, "b2"),
					Body(r, "b3"),
					Body(r, "b4")};

	std::vector<Joint> joints = {Joint(Joint::RevX, true, "j0"),
					Joint(Joint::RevX, true, "j1"),
					Joint(Joint::RevX, true, "j2"),
					Joint(Joint::RevX, true, "j3")};

	std::vector<int> pred = {-1, 0, 0, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 0, 2};
	Vector3d tmp = Vector3d::Random();

	PTransformd I = PTransformd::Identity();
	std::vector<PTransformd> Xt = {I, I, PTransformd(tmp), I};

	MultiBody mb(bodies, joints, pred, succ, parent, Xt);

	// check MultiBody equality
	checkMultiBodyEq(mb, bodies, joints, pred, succ, parent, Xt);

	// NameToIndex
	// bodyIndexByName
	BOOST_CHECK_EQUAL(mb.bodyIndexByName("b1"), 0);
	BOOST_CHECK_EQUAL(mb.bodyIndexByName("b2"), 1);
	BOOST_CHECK_EQUAL(mb.bodyIndexByName("b3"), 2);
	BOOST_CHECK_EQUAL(mb.bodyIndexByName("b4"), 3);

	// jointIndexByName
	BOOST_CHECK_EQUAL(mb.jointIndexByName("j0"), 0);
	BOOST_CHECK_EQUAL(mb.jointIndexByName("j1"), 1);
	BOOST_CHECK_EQUAL(mb.jointIndexByName("j2"), 2);
	BOOST_CHECK_EQUAL(mb.jointIndexByName("j3"), 3);

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
	BOOST_CHECK_NO_THROW(mb.sBodyIndexByName("b1"));
	BOOST_CHECK_THROW(mb.sBodyIndexByName("b10"), std::out_of_range);

	// jointIndexById
	BOOST_CHECK_NO_THROW(mb.sJointIndexByName("j0"));
	BOOST_CHECK_THROW(mb.sJointIndexByName("j10"), std::out_of_range);


	// Setter test


	// transform setter
	sva::PTransformd newTrans(Vector3d(1., 1., 1));
	BOOST_CHECK_NO_THROW(mb.sTransform(1, newTrans));
	BOOST_CHECK_THROW(mb.sTransform(10, newTrans), std::out_of_range);
	BOOST_CHECK_EQUAL(mb.transform(1), newTrans);
	mb.transform(0, newTrans);
	BOOST_CHECK_EQUAL(mb.transform(0), newTrans);

	// transforms setter
	std::vector<PTransformd> newXt = {newTrans, newTrans, I, tmp};
	BOOST_CHECK_NO_THROW(mb.sTransforms(newXt));
	BOOST_CHECK_THROW(mb.sTransforms({newTrans, newTrans, I}), std::runtime_error);
	checkMultiBodyEq(mb, bodies, joints, pred, succ, parent, newXt);

	// body setter
	int b3Index = mb.bodyIndexByName("b3");
	Body oldBody(mb.body(b3Index));
	RBInertiad newR(123., Vector3d::Random(), Matrix3d::Random());
	Body newBody(newR, "b3");
	BOOST_CHECK_NO_THROW(mb.sBody(b3Index, newBody));
	BOOST_CHECK_THROW(mb.sBody(10, newBody), std::out_of_range);
	BOOST_CHECK_EQUAL(mb.body(b3Index), newBody);
	mb.body(b3Index, oldBody);
	BOOST_CHECK_EQUAL(mb.body(b3Index), oldBody);

	// bodies setter
	std::vector<Body> newBodies = {Body(newR, "b1"), Body(newR, "b2"),
		Body(newR, "b3"), Body(r, "b4")};
	BOOST_CHECK_NO_THROW(mb.sBodies(newBodies));
	BOOST_CHECK_THROW(mb.sBodies({Body(newR, "b1"), Body(newR, "b2")}),
		std::runtime_error);
	checkMultiBodyEq(mb, bodies, joints, pred, succ, parent, newXt);
}



BOOST_AUTO_TEST_CASE(MakeMultiBodyTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;
	namespace constants = boost::math::constants;

	MultiBodyGraph mbg1;

	RBInertiad rbi(1., Vector3d::Zero(), Matrix3d::Identity());
	Body b1(rbi, "b1");
	Body b2(rbi, "b2");
	Body b3(rbi, "b3");
	Body b4(rbi, "b4");
	mbg1.addBody(b1);
	mbg1.addBody(b2);
	mbg1.addBody(b3);
	mbg1.addBody(b4);

	Joint j1(Joint::RevX, true, "j1");
	Joint j2(Joint::RevX, true, "j2");
	Joint j3(Joint::RevX, true, "j3");
	mbg1.addJoint(j1);
	mbg1.addJoint(j2);
	mbg1.addJoint(j3);

	//        b2
	//   j1  /   j2      j3
	//       b1 ---- b3 ---- b4
	mbg1.linkBodies("b1", PTransformd::Identity(), "b2", PTransformd::Identity(), "j1");
	mbg1.linkBodies("b1", PTransformd::Identity(), "b3", PTransformd::Identity(), "j2");
	mbg1.linkBodies("b3", PTransformd::Identity(), "b4", PTransformd::Identity(), "j3");


	//                b2
	//           j1  /  j2      j3
	// root -fixed- b1 ---- b3 ---- b4

	BOOST_CHECK_NO_THROW(mbg1.makeMultiBody("b1", true));
	BOOST_CHECK_THROW(mbg1.makeMultiBody("b10", true), std::out_of_range);

	// test the successorJoints and predecessorJoint function
	auto testSucc = [](const std::map<std::string, std::vector<std::string>>& s1,
			const std::map<std::string, std::vector<std::string>>& s2)
	{
		BOOST_CHECK_EQUAL(s1.size(), s2.size());
		auto it1 = s1.begin();
		auto it2 = s2.begin();
		for(; it1 != s1.end(); ++it1, ++it2)
		{
			BOOST_CHECK_EQUAL(it1->first, it2->first);
			BOOST_CHECK_EQUAL_COLLECTIONS(it1->second.begin(), it1->second.end(),
				it2->second.begin(), it2->second.end());
		}
	};
	auto testPred = [](const std::map<std::string, std::string>& s1,
			const std::map<std::string, std::string>& s2)
	{
		BOOST_CHECK_EQUAL(s1.size(), s2.size());
		auto it1 = s1.begin();
		auto it2 = s2.begin();
		for(; it1 != s1.end(); ++it1, ++it2)
		{
			BOOST_CHECK_EQUAL(it1->first, it2->first);
			BOOST_CHECK_EQUAL(it1->second, it2->second);
		}
	};
	std::map<std::string, std::vector<std::string>> succRes1{
			{"b1",{"j1","j2"}},
			{"b2",{}},
			{"b3",{"j3"}},
			{"b4",{}}};
	std::map<std::string, std::string> predRes1{
			{"b1","Root"},
			{"b2","j1"},
			{"b3","j2"},
			{"b4","j3"}};

	testSucc(mbg1.successorJoints("b1"), succRes1);
	testPred(mbg1.predecessorJoint("b1"), predRes1);

	MultiBody mb1 = mbg1.makeMultiBody("b1", true);


	std::vector<Body> bodies = {b1, b2, b3, b4};

	std::vector<Joint> joints = {Joint(Joint::Fixed, true, "Root"),
		j1, j2, j3};

	std::vector<int> pred = {-1, 0, 0, 2};
	std::vector<int> succ = {0, 1, 2, 3};
	std::vector<int> parent = {-1, 0, 0, 2};

	PTransformd I = PTransformd::Identity();
	std::vector<PTransformd> Xt = {I, I, I, I};

	// check MultiBody equality
	checkMultiBodyEq(mb1, bodies, joints, pred, succ, parent, Xt);

	// check bodyIndexById
	BOOST_CHECK_EQUAL(mb1.bodyIndexByName("b1"), 0);
	BOOST_CHECK_EQUAL(mb1.bodyIndexByName("b2"), 1);
	BOOST_CHECK_EQUAL(mb1.bodyIndexByName("b3"), 2);
	BOOST_CHECK_EQUAL(mb1.bodyIndexByName("b4"), 3);

	// check jointIndexById
	BOOST_CHECK_EQUAL(mb1.jointIndexByName("Root"), 0);
	BOOST_CHECK_EQUAL(mb1.jointIndexByName("j1"), 1);
	BOOST_CHECK_EQUAL(mb1.jointIndexByName("j2"), 2);
	BOOST_CHECK_EQUAL(mb1.jointIndexByName("j3"), 3);



	//                b2
	//          j1  /  j2      j3
	// root -free- b1 ---- b3 ---- b4

	MultiBody mb2 = mbg1.makeMultiBody("b1", false);
	joints = {Joint(Joint::Free, true, "Root"), j1, j2, j3};

	// check MultiBody equality
	checkMultiBodyEq(mb2, bodies, joints, pred, succ, parent, Xt);

	// test the successorJoints and predecessorJoint function
	std::map<std::string, std::vector<std::string>> succRes2{
			{"b1", {"j1","j2"}},
			{"b2", {}},
			{"b3", {"j3"}},
			{"b4", {}}};
	std::map<std::string, std::string> predRes2{
			{"b1", "Root"},
			{"b2", "j1"},
			{"b3", "j2"},
			{"b4", "j3"}};

	testSucc(mbg1.successorJoints("b1"), succRes2);
	testPred(mbg1.predecessorJoint("b1"), predRes2);



	//                   j1      j2      j3
	// root -fixed-  b2 ---- b1 ---- b3 ---- b4

	MultiBody mb3 = mbg1.makeMultiBody("b2", true);

	// test the successorJoints and predecessorJoint function
	std::map<std::string, std::vector<std::string>> succRes3{
			{"b2", {"j1"}},
			{"b1", {"j2"}},
			{"b3", {"j3"}},
			{"b4", {}}};
	std::map<std::string, std::string> predRes3{
			{"b2", "Root"},
			{"b1", "j1"},
			{"b3", "j2"},
			{"b4", "j3"}};

	testSucc(mbg1.successorJoints("b2"), succRes3);
	testPred(mbg1.predecessorJoint("b2"), predRes3);

	bodies = {b2, b1, b3, b4};

	j1.forward(false);
	joints = {Joint(Joint::Fixed, true, "Root"), j1, j2, j3};

	pred = {-1, 0, 1, 2};
	succ = {0, 1, 2, 3};
	parent = {-1, 0, 1, 2};

	Xt = {I, I, I, I};

	// check joint j1 direction
	BOOST_CHECK_EQUAL(mb3.joint(1).forward(), false);

	// check MultiBody equality
	checkMultiBodyEq(mb3, bodies, joints, pred, succ, parent, Xt);

	// check bodyIndexByName
	BOOST_CHECK_EQUAL(mb3.bodyIndexByName("b1"), 1);
	BOOST_CHECK_EQUAL(mb3.bodyIndexByName("b2"), 0);
	BOOST_CHECK_EQUAL(mb3.bodyIndexByName("b3"), 2);
	BOOST_CHECK_EQUAL(mb3.bodyIndexByName("b4"), 3);

	// check jointIndexByName
	BOOST_CHECK_EQUAL(mb3.jointIndexByName("Root"), 0);
	BOOST_CHECK_EQUAL(mb3.jointIndexByName("j1"), 1);
	BOOST_CHECK_EQUAL(mb3.jointIndexByName("j2"), 2);
	BOOST_CHECK_EQUAL(mb3.jointIndexByName("j3"), 3);



	// check transform                         j2
	//                       j1           tx(1)-tz(-1) --- B3
	// root -fixed- B1 -tx(1)-ty(-1)- B2 /     j3
	//                                   \ rx(90)-I ---- B4

	MultiBodyGraph mbg2;

	mbg2.addBody(b1);
	mbg2.addBody(b2);
	mbg2.addBody(b3);
	mbg2.addBody(b4);

	mbg2.addJoint(j1);
	mbg2.addJoint(j2);
	mbg2.addJoint(j3);

	mbg2.linkBodies("b1", PTransformd(Vector3d(Vector3d::UnitX())),
			"b2", PTransformd(Vector3d(-Vector3d::UnitY())), "j1");
	mbg2.linkBodies("b2", PTransformd(Vector3d(Vector3d::UnitX())),
			"b3", PTransformd(Vector3d(-Vector3d::UnitZ())), "j2");
	mbg2.linkBodies("b2", PTransformd(Matrix3d(sva::RotX(constants::pi<double>()/2.))),
			"b4", PTransformd::Identity(), "j3");

	// add () around the Vector3d because Clang think that
	// a function declaration
	sva::PTransformd root((Vector3d(Vector3d::Random())));
	MultiBody mb4 = mbg2.makeMultiBody("b1", true, root);

	// test the successorJoints and predecessorJoint function
	std::map<std::string, std::vector<std::string>> succRes4{
			{"b1", {"j1"}},
			{"b2", {"j2", "j3"}},
			{"b3", {}},
			{"b4", {}}};
	std::map<std::string, std::string> predRes4{
			{"b1", "Root"},
			{"b2", "j1"},
			{"b3", "j2"},
			{"b4", "j3"}};

	testSucc(mbg2.successorJoints("b1"), succRes4);
	testPred(mbg2.predecessorJoint("b1"), predRes4);


	bodies = {b1, b2, b3, b4};

	joints = {Joint(Joint::Fixed, true, "Root"), j1, j2, j3};

	pred = {-1, 0, 1, 1};
	succ = {0, 1, 2, 3};
	parent = {-1, 0, 1, 1};

	Xt = {root,
				PTransformd(Vector3d(1., 0., 0.)),
				PTransformd(Vector3d(1., 1., 0.)),
				PTransformd(RotX(constants::pi<double>()/2.), Vector3d(0., 1., 0.))};

	// check MultiBody equality
	checkMultiBodyEq(mb4, bodies, joints, pred, succ, parent, Xt);


	// check body com
	std::vector<Vector3d> bCom = {Vector3d::Zero(),
																Vector3d(0., 1., 0.),
																Vector3d(0., 0., 1.),
																Vector3d(0., 0., 0.)};

	for(int i = 0; i < mb4.nrBodies(); ++i)
	{
		BOOST_CHECK_EQUAL(mb4.body(i).inertia().momentum(), bCom[i]);
	}
}


BOOST_AUTO_TEST_CASE(MultiBodyConfigFunction)
{
	using namespace std;
	using namespace Eigen;
	using namespace rbd;

	std::vector<std::vector<double>> v1 = {{}, {1., 2., 3.}, {4.}, {5., 6.}, {7.}};
	std::vector<std::vector<double>> v2 = {{}, {0., 0., 0.}, {0.}, {0., 0.}, {0.}};
	VectorXd e(7);

	BOOST_CHECK_NO_THROW(sParamToVector(v1, e));
	BOOST_CHECK_NO_THROW(sVectorToParam(e, v2));

	for(std::size_t i = 0; i < v1.size(); ++i)
	{
		BOOST_CHECK_EQUAL_COLLECTIONS(v1[i].begin(), v1[i].end(),
			v2[i].begin(), v2[i].end());
	}

	e.resize(4);
	BOOST_CHECK_THROW(sParamToVector(v1, e), out_of_range);
	BOOST_CHECK_THROW(sVectorToParam(e, v2), out_of_range);
}


BOOST_AUTO_TEST_CASE(MultiBodyConfigFunction2)
{
	using namespace std;
	using namespace Eigen;
	using namespace rbd;
	using namespace sva;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	Body b0(rbi, "b0");
	Body b1(rbi, "b1");
	Body b2(rbi, "b2");
	Body b3(rbi, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::Spherical, true, "j0");
	Joint j1(Joint::Spherical, true, "j1");
	Joint j2(Joint::RevX, true, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies("b0", PTransformd::Identity(), "b1", PTransformd::Identity(), "j0");
	mbg.linkBodies("b1", PTransformd::Identity(), "b2", PTransformd::Identity(), "j1");
	mbg.linkBodies("b2", PTransformd::Identity(), "b3", PTransformd::Identity(), "j2");

	MultiBody mb = mbg.makeMultiBody("b0", true);

	MultiBodyConfig mbc(mb);

	// test paramToVector
	vector<vector<double>> confP = {{}, {1., 0., 0., 0.}, {1., 0., 0., 0.}, {2.}};
	vector<vector<double>> confD = {{}, {1., 2., 3.}, {3., 4., 5.}, {7.}};
	VectorXd eTestP(9);
	VectorXd eTestD(7);
	VectorXd e;

	paramToVector(confP, eTestP);
	paramToVector(confD, eTestD);

	BOOST_CHECK_NO_THROW(e = sParamToVector(mb, confP));
	BOOST_CHECK_THROW(sParamToVector(mb, confD), std::out_of_range);

	BOOST_CHECK_EQUAL(e, eTestP);



	// test dofToVector
	BOOST_CHECK_NO_THROW(e = sDofToVector(mb, confD));
	BOOST_CHECK_THROW(sDofToVector(mb, confP), std::out_of_range);

	BOOST_CHECK_EQUAL(e, eTestD);



	// test vectorToParam
	vector<vector<double>> conf;
	BOOST_CHECK_NO_THROW(conf = sVectorToParam(mb, eTestP));
	BOOST_CHECK_THROW(sVectorToParam(mb, eTestD), std::out_of_range);

	BOOST_CHECK_EQUAL(conf.size(), confP.size());
	for(std::size_t i = 0; i < conf.size(); ++i)
	{
		BOOST_CHECK_EQUAL_COLLECTIONS(conf[i].begin(), conf[i].end(),
			confP[i].begin(), confP[i].end());
	}



	// test vectorToDof
	BOOST_CHECK_NO_THROW(conf = sVectorToDof(mb, eTestD));
	BOOST_CHECK_THROW(sVectorToDof(mb, eTestP), std::out_of_range);

	BOOST_CHECK_EQUAL(conf.size(), confD.size());
	for(std::size_t i = 0; i < conf.size(); ++i)
	{
		BOOST_CHECK_EQUAL_COLLECTIONS(conf[i].begin(), conf[i].end(),
			confD[i].begin(), confD[i].end());
	}

	// test zero configuration
	mb = mbg.makeMultiBody("b0", false);

	mbc = MultiBodyConfig(mb);
	mbc.zero(mb);
	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		std::vector<double> zp = mb.joint(i).zeroParam();
		std::vector<double> zd = mb.joint(i).zeroDof();

		BOOST_CHECK_EQUAL_COLLECTIONS(mbc.q[i].begin(), mbc.q[i].end(),
																	zp.begin(), zp.end());
		BOOST_CHECK_EQUAL_COLLECTIONS(mbc.alpha[i].begin(), mbc.alpha[i].end(),
																	zd.begin(), zd.end());
		BOOST_CHECK_EQUAL_COLLECTIONS(mbc.alphaD[i].begin(), mbc.alphaD[i].end(),
																	zd.begin(), zd.end());
		BOOST_CHECK_EQUAL_COLLECTIONS(mbc.jointTorque[i].begin(), mbc.jointTorque[i].end(),
																	zd.begin(), zd.end());
	}

	for(int i = 0; i < mb.nrBodies(); ++i)
	{
		BOOST_CHECK_EQUAL(mbc.force[i].vector(), Vector6d::Zero());
	}
}


void checkConfig(const rbd::MultiBodyConfig& mbc1, const rbd::MultiBodyConfig& mbc2)
{
	for(std::size_t i = 0; i < mbc1.q.size(); ++i)
	{
		BOOST_CHECK_EQUAL_COLLECTIONS(mbc1.q[i].begin(), mbc1.q[i].end(),
			mbc2.q[i].begin(), mbc2.q[i].end());

		BOOST_CHECK_EQUAL_COLLECTIONS(mbc1.alpha[i].begin(), mbc1.alpha[i].end(),
			mbc2.alpha[i].begin(), mbc2.alpha[i].end());

		BOOST_CHECK_EQUAL_COLLECTIONS(mbc1.alphaD[i].begin(), mbc1.alphaD[i].end(),
			mbc2.alphaD[i].begin(), mbc2.alphaD[i].end());
	}

	BOOST_CHECK_EQUAL_COLLECTIONS(mbc1.force.begin(), mbc1.force.end(),
		mbc2.force.begin(), mbc2.force.end());
}

BOOST_AUTO_TEST_CASE(ConfigConverterTest)
{
	using namespace std;
	using namespace Eigen;
	using namespace rbd;
	using namespace sva;

	MultiBodyGraph mbg;

	double mass = 1.;
	Matrix3d I = Matrix3d::Identity();
	Vector3d h = Vector3d::Zero();

	RBInertiad rbi(mass, h, I);

	Body b0(rbi, "b0");
	Body b1(rbi, "b1");
	Body b2(rbi, "b2");
	Body b3(rbi, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::Spherical, true, "j0");
	Joint j1(Joint::RevY, true, "j1");
	Joint j2(Joint::RevX, true, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies("b0", PTransformd::Identity(), "b1", PTransformd::Identity(), "j0");
	mbg.linkBodies("b1", PTransformd::Identity(), "b2", PTransformd::Identity(), "j1");
	mbg.linkBodies("b2", PTransformd::Identity(), "b3", PTransformd::Identity(), "j2");

	MultiBody mb1 = mbg.makeMultiBody("b0", true);
	MultiBody mb2 = mbg.makeMultiBody("b3", true);
	MultiBody mb3 = mbg.makeMultiBody("b1", true);

	ConfigConverter* mb1tomb2 = ConfigConverter::sConstructor(mb1, mb2);
	ConfigConverter* mb1tomb3 = ConfigConverter::sConstructor(mb1, mb3);

	ConfigConverter* mb2tomb1 = ConfigConverter::sConstructor(mb2, mb1);
	ConfigConverter* mb3tomb1 = ConfigConverter::sConstructor(mb3, mb1);


	MultiBodyConfig mbc1(mb1);
	MultiBodyConfig mbc1Tmp(mb1);
	MultiBodyConfig mbc2(mb2);
	MultiBodyConfig mbc3(mb3);

	mbc1.q = {{}, {1., 0., 0., 0.}, {1.}, {2.}};
	mbc1.alpha = {{}, {1., 0., 0., 0.}, {1.2}, {2.2}};
	mbc1.alphaD = {{}, {0., 3., 2.}, {1.4}, {2.3}};
	mbc1.force = {ForceVecd(Vector6d::Random()), ForceVecd(Vector6d::Random()),
								ForceVecd(Vector6d::Random()), ForceVecd(Vector6d::Random())};

	mb1tomb2->sConvert(mbc1, mbc2);
	mb2tomb1->sConvert(mbc2, mbc1Tmp);
	checkConfig(mbc1, mbc1Tmp);

	mbc1.q = {{}, {1., 0., 0., 0.}, {5.}, {6.}};
	mb1tomb3->sConvert(mbc1, mbc3);
	mb3tomb1->sConvert(mbc3, mbc1Tmp);
	checkConfig(mbc1, mbc1Tmp);


	// sConvertJoint
	mbc1.q = {{}, {1., 0., 0., 0.}, {5.}, {3.}};
	mb1tomb2->sConvertJoint(mbc1.q, mbc2.q);
	mb2tomb1->sConvertJoint(mbc2.q, mbc1Tmp.q);
	checkConfig(mbc1, mbc1Tmp);

	mbc1.q = {{}, {1., 0., 0., 0.}, {2.}, {3.}};
	mb1tomb3->sConvertJoint(mbc1.q, mbc3.q);
	mb3tomb1->sConvertJoint(mbc3.q, mbc1Tmp.q);
	checkConfig(mbc1, mbc1Tmp);

	// sConvertJoint return version
	mbc1.q = {{}, {1., 0., 0., 0.}, {5.}, {3.}};
	mbc2.q = mb1tomb2->convertJoint(mbc1.q);
	mbc1Tmp.q = mb2tomb1->convertJoint(mbc2.q);
	checkConfig(mbc1, mbc1Tmp);

	mbc1.q = {{}, {1., 0., 0., 0.}, {2.}, {3.}};
	mbc3.q = mb1tomb3->convertJoint(mbc1.q);
	mbc1Tmp.q = mb3tomb1->convertJoint(mbc3.q);
	checkConfig(mbc1, mbc1Tmp);


	delete mb1tomb2;
	delete mb1tomb3;
	delete mb2tomb1;
	delete mb3tomb1;
}


BOOST_AUTO_TEST_CASE(MultiBodyBaseTransformTest)
{
	using namespace Eigen;

	// we construct 3 multibody from body id 0, 3 and 4
	// we try to find the base of each body with all the multibody
	rbd::MultiBody mb0, mb3, mb4;
	rbd::MultiBodyConfig mbc0, mbc3, mbc4;
	rbd::MultiBodyGraph mbg;
	sva::PTransformd mb3Surface(Quaterniond(Vector4d::Random()).normalized(),
					Vector3d::Random());
	sva::PTransformd mb4Surface(Quaterniond(Vector4d::Random()).normalized(),
					Vector3d::Random());
	std::tie(mb0, mbc0, mbg) = makeXYZSarm();
	// the multibody graph is construct with the old body linking api
	// so even the multibody from body 0 must have transform to each bodies base
	auto mb0ToBase = mbg.bodiesBaseTransform("b0");

	VectorXd q(VectorXd::Random(mb0.nrParams()));
	// we must normalize the configuration of the spherical joint
	q.segment(mb0.jointPosInParam(mb0.jointIndexByName("j3")), 4).normalize();
	rbd::vectorToParam(q, mbc0.q);
	rbd::forwardKinematics(mb0, mbc0);

	// since the multibody is construct with the old body linking api
	// we must add an offset to joint origin of mb3 that correspond mb0 transform
	// to body base
	mb3 = mbg.makeMultiBody("b3", true, mb3Surface*mb0ToBase["b3"]*mbc0.bodyPosW[mb0.bodyIndexByName("b3")],
			mb3Surface);
	rbd::ConfigConverter mb0ToMb3(mb0, mb3);
	mbc3 = rbd::MultiBodyConfig(mb3);
	mb0ToMb3.convert(mbc0, mbc3);
	rbd::forwardKinematics(mb3, mbc3);
	auto mb3ToBase = mbg.bodiesBaseTransform("b3", mb3Surface);

	mb4 = mbg.makeMultiBody("b4", true, mb4Surface*mb0ToBase["b4"]*mbc0.bodyPosW[mb0.bodyIndexByName("b4")],
			mb4Surface);
	mbc4 = rbd::MultiBodyConfig(mb4);
	rbd::ConfigConverter mb0ToMb4(mb0, mb4);
	mb0ToMb4.convert(mbc0, mbc4);
	rbd::forwardKinematics(mb4, mbc4);
	auto mb4ToBase = mbg.bodiesBaseTransform("b4", mb4Surface);

	for(int bi = 0; bi < mb0.nrBodies(); ++bi)
	{
		std::string name = mb0.body(bi).name();
		int mb3Index = mb3.bodyIndexByName(name);
		int mb4Index = mb4.bodyIndexByName(name);
		BOOST_CHECK_SMALL(((mb0ToBase[name]*mbc0.bodyPosW[bi]).matrix() -
											 (mb3ToBase[name]*mbc3.bodyPosW[mb3Index]).matrix()).norm(),
											1e-8);
		BOOST_CHECK_SMALL(((mb0ToBase[name]*mbc0.bodyPosW[bi]).matrix() -
											 (mb4ToBase[name]*mbc4.bodyPosW[mb4Index]).matrix()).norm(),
											1e-8);
	}
}
