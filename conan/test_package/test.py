#!/usr/bin/env python
# -*- coding: utf-8 -*-

import eigen
import sva
import rbdyn

mbg = rbdyn.MultiBodyGraph()

mass = 1.0
I = eigen.Matrix3d.Identity()
h = eigen.Vector3d.Zero()

rbi = sva.RBInertiad(mass, h, I)

b0 = rbdyn.Body(rbi, "b0")
b1 = rbdyn.Body(rbi, "b1")
b2 = rbdyn.Body(rbi, "b2")
b3 = rbdyn.Body(rbi, "b3")

mbg.addBody(b0)
mbg.addBody(b1)
mbg.addBody(b2)
mbg.addBody(b3)

j0 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitX(), True, "j0")
j1 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitX(), True, "j1")
j2 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitX(), True, "j2")

mbg.addJoint(j0)
mbg.addJoint(j1)
mbg.addJoint(j2)

to = sva.PTransformd(eigen.Vector3d.UnitY())
from_ = sva.PTransformd(eigen.Vector3d.Zero())

mbg.linkBodies("b0", from_, "b1", from_, "j0")
mbg.linkBodies("b1", to, "b2", from_, "j1")
mbg.linkBodies("b2", to, "b3", from_, "j2")

mb = mbg.makeMultiBody("b0", True)
mbc = rbdyn.MultiBodyConfig(mb)
mbc.zero(mb);

rbdyn.forwardKinematics(mb, mbc)
print(mbc.bodyPosW[-1])
