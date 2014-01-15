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
#define BOOST_TEST_MODULE MultiBodyTest
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "Body.h"
#include "Joint.h"
#include "MultiBody.h"
#include "MultiBodyGraph.h"
#include "MultiBodyConfig.h"

// arm
#include "XYZSarm.h"


BOOST_AUTO_TEST_CASE(MultiBodyGraphTest)
{
	using namespace Eigen;
	using namespace sva;
	using namespace rbd;

	MultiBodyGraph mbg1;

	// test addBody
	Body b1(RBInertiad(), 0, "b1");
	Body b2(RBInertiad(), 1, "b2");
	Body b3(RBInertiad(), 2, "b3");
	Body b4(RBInertiad(), 3, "b4");
	BOOST_CHECK_NO_THROW(mbg1.addBody(b1));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b2));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b3));
	BOOST_CHECK_NO_THROW(mbg1.addBody(b4));

	// id already exist
	BOOST_CHECK_THROW(mbg1.addBody(Body(RBInertiad(), 0, "b3")), std::domain_error);

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

	// name already exist
	BOOST_CHECK_THROW(mbg1.addJoint(Joint(Joint::RevX, true, 3, "j1")), std::domain_error);

	// must be 3 joints
	BOOST_CHECK_EQUAL(mbg1.nrJoints(), 3);

	// test bodyIdByName
	BOOST_CHECK_EQUAL(mbg1.bodyIdByName("b1"), 0);
	BOOST_CHECK_EQUAL(mbg1.bodyIdByName("b2"), 1);
	BOOST_CHECK_EQUAL(mbg1.bodyIdByName("b3"), 2);
	BOOST_CHECK_EQUAL(mbg1.bodyIdByName("b4"), 3);

	// test jointById
	std::shared_ptr<Joint> joint1, joint2, joint3;
	BOOST_CHECK_NO_THROW(joint1 = mbg1.jointById(0));
	BOOST_CHECK_NO_THROW(joint2 = mbg1.jointById(1));
	BOOST_CHECK_NO_THROW(joint3 = mbg1.jointById(2));

	BOOST_CHECK_EQUAL(*joint1, j1);
	BOOST_CHECK_EQUAL(*joint2, j2);
	BOOST_CHECK_EQUAL(*joint3, j3);

	// test jointByName
	// test also jointIdByName since jointByName use jointIdByName
	BOOST_CHECK_NO_THROW(joint1 = mbg1.jointByName("j1"));
	BOOST_CHECK_NO_THROW(joint2 = mbg1.jointByName("j2"));
	BOOST_CHECK_NO_THROW(joint3 = mbg1.jointByName("j3"));

	BOOST_CHECK_EQUAL(*joint1, j1);
	BOOST_CHECK_EQUAL(*joint2, j2);
	BOOST_CHECK_EQUAL(*joint3, j3);

	// check non-existant id
	BOOST_CHECK_THROW(mbg1.jointById(10), std::out_of_range);

	// check non-existant name
	BOOST_CHECK_THROW(mbg1.jointByName("j10"), std::out_of_range);

	// test linkBody
	//        b2(1)
	//   j1(0)  /  j2(1)     j3(2)
	//       b1(0) ---- b3(2) ---- b4(3)

	BOOST_CHECK_NO_THROW(mbg1.linkBodies(0, PTransformd::Identity(),
		1, PTransformd::Identity(), 0));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies(0, PTransformd::Identity(),
		2, PTransformd::Identity(), 1));
	BOOST_CHECK_NO_THROW(mbg1.linkBodies(2, PTransformd::Identity(),
		3, PTransformd::Identity(), 2));

	// check non-existant body 1
	BOOST_CHECK_THROW(mbg1.linkBodies(10, PTransformd::Identity(),
		3, PTransformd::Identity(), 2), std::out_of_range);
	// check non-existant body 2
	BOOST_CHECK_THROW(mbg1.linkBodies(2, PTransformd::Identity(),
		10, PTransformd::Identity(), 2), std::out_of_range);
	// check non-existant joint
	BOOST_CHECK_THROW(mbg1.linkBodies(2, PTransformd::Identity(),
		3, PTransformd::Identity(), 10), std::out_of_range);
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
	rbd::MultiBodyGraph mbg, mbgBack;
	std::tie(mb, mbc, mbg) = makeXYZSarm();


	BOOST_CHECK_EQUAL(mbg.nrJoints(), 4);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 5);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 5);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 5);

	mbg.removeJoint(0, "j3");

	mb = mbg.makeMultiBody(0, true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 4);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 4);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 4);
	checkMultiBodyNames(mb, {"b0", "b1", "b2", "b3"}, {"Root", "j0", "j1", "j2"});


	std::tie(mb, mbc, mbg) = makeXYZSarm();

	mbg.removeJoint(0, "j0");
	mb = mbg.makeMultiBody(0, true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 0);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 1);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 1);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 1);

	BOOST_CHECK_EQUAL(mb.body(0).name(), "b0");
	BOOST_CHECK_EQUAL(mb.joint(0).name(), "Root");


	std::tie(mb, mbc, mbg) = makeXYZSarm();

	mbg.removeJoints(0, std::vector<std::string>({"j3", "j2"}));
	mb = mbg.makeMultiBody(0, true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 2);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 3);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 3);
	checkMultiBodyNames(mb, {"b0", "b1", "b2"}, {"Root", "j0", "j1"});


	std::tie(mb, mbc, mbg) = makeXYZSarm();

	mbg.removeJoint(0, 1);
	mb = mbg.makeMultiBody(0, true);
	BOOST_CHECK_EQUAL(mbg.nrJoints(), 2);
	BOOST_CHECK_EQUAL(mbg.nrNodes(), 3);
	BOOST_CHECK_EQUAL(mb.nrJoints(), 3);
	BOOST_CHECK_EQUAL(mb.nrBodies(), 3);
	checkMultiBodyNames(mb, {"b0", "b1", "b4"}, {"Root", "j0", "j3"});


	std::tie(mb, mbc, mbg) = makeXYZSarm();

	mbg.removeJoints(0, std::vector<int>({1, 2}));
	mb = mbg.makeMultiBody(0, true);
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
	//                   b2(1)
	//             j1(0)  /  j2(1)     j3(2)
	// root --j0(-1)-- b1(0) ---- b3(2) ---- b4(3)

	RBInertiad r;
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

	PTransformd I = PTransformd::Identity();
	std::vector<PTransformd> Xt = {I, I, PTransformd(tmp), I};

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

	RBInertiad rbi(1., Vector3d::Zero(), Matrix3d::Identity());
	Body b1(rbi, 0, "b1");
	Body b2(rbi, 1, "b2");
	Body b3(rbi, 2, "b3");
	Body b4(rbi, 3, "b4");
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
	mbg1.linkBodies(0, PTransformd::Identity(), 1, PTransformd::Identity(), 0);
	mbg1.linkBodies(0, PTransformd::Identity(), 2, PTransformd::Identity(), 1);
	mbg1.linkBodies(2, PTransformd::Identity(), 3, PTransformd::Identity(), 2);


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

	PTransformd I = PTransformd::Identity();
	std::vector<PTransformd> Xt = {I, I, I, I};

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

	mbg2.linkBodies(0, PTransformd(Vector3d(Vector3d::UnitX())),
									1, PTransformd(Vector3d(-Vector3d::UnitY())), 0);
	mbg2.linkBodies(1, PTransformd(Vector3d(Vector3d::UnitX())),
									2, PTransformd(Vector3d(-Vector3d::UnitZ())), 1);
	mbg2.linkBodies(1, PTransformd(Matrix3d(sva::RotX(constants::pi<double>()/2.))),
									3, PTransformd::Identity(), 2);

	sva::PTransformd root(Vector3d(Vector3d::Random()));
	MultiBody mb4 = mbg2.makeMultiBody(0, true, root);


	bodies = {b1, b2, b3, b4};

	joints = {Joint(Joint::Fixed, true, -1, "Root"), j1, j2, j3};

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

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(rbi, 2, "b2");
	Body b3(rbi, 3, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::Spherical, true, 0, "j0");
	Joint j1(Joint::Spherical, true, 1, "j1");
	Joint j2(Joint::RevX, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies(0, PTransformd::Identity(), 1, PTransformd::Identity(), 0);
	mbg.linkBodies(1, PTransformd::Identity(), 2, PTransformd::Identity(), 1);
	mbg.linkBodies(2, PTransformd::Identity(), 3, PTransformd::Identity(), 2);

	MultiBody mb = mbg.makeMultiBody(0, true);

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
	mb = mbg.makeMultiBody(0, false);

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

	Body b0(rbi, 0, "b0");
	Body b1(rbi, 1, "b1");
	Body b2(rbi, 2, "b2");
	Body b3(rbi, 3, "b3");

	mbg.addBody(b0);
	mbg.addBody(b1);
	mbg.addBody(b2);
	mbg.addBody(b3);

	Joint j0(Joint::Spherical, true, 0, "j0");
	Joint j1(Joint::RevY, true, 1, "j1");
	Joint j2(Joint::RevX, true, 2, "j2");

	mbg.addJoint(j0);
	mbg.addJoint(j1);
	mbg.addJoint(j2);

	mbg.linkBodies(0, PTransformd::Identity(), 1, PTransformd::Identity(), 0);
	mbg.linkBodies(1, PTransformd::Identity(), 2, PTransformd::Identity(), 1);
	mbg.linkBodies(2, PTransformd::Identity(), 3, PTransformd::Identity(), 2);

	MultiBody mb1 = mbg.makeMultiBody(0, true);
	MultiBody mb2 = mbg.makeMultiBody(3, true);
	MultiBody mb3 = mbg.makeMultiBody(1, true);

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
