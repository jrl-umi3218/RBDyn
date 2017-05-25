# -*- coding: utf-8 -*-
#
# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# RBDyn is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RBDyn is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

import unittest

import pickle

import eigen as e3
import sva
import rbdyn as rbd

class TestRBDynPickle(unittest.TestCase):
  def test(self):
    # regiter custom pickle function
    rbd.copy_reg_pickle()

    # create a body with random inertia
    def makeBody(bName):
      I = sva.RBInertiad(e3.Vector3d.Random().x(),
                         e3.Vector3d.Random(), e3.Matrix3d.Random())
      return rbd.Body(I, bName)

    body = makeBody('testBody')

    jR = rbd.Joint(rbd.Joint.Rev, e3.Vector3d.Random().normalized(), True, 'jR')
    jP = rbd.Joint(rbd.Joint.Prism, e3.Vector3d.Random().normalized(),
                   False, 'jP')
    jS = rbd.Joint(rbd.Joint.Spherical, False, 'jS')
    jPla = rbd.Joint(rbd.Joint.Planar, True, 'jPla')
    jC = rbd.Joint(rbd.Joint.Cylindrical, e3.Vector3d.Random().normalized(),
                   False, 'jC')
    jFree = rbd.Joint(rbd.Joint.Free, True, 'jFree')
    jFix = rbd.Joint(rbd.Joint.Fixed, False, 'jFix')

    mb = rbd.MultiBody([makeBody('body%s' % i) for i in range(7)],
                       [jR, jP, jS, jPla, jC, jFree, jFix],
                       list(range(-1, 6)), list(range(0, 7)), list(range(-1, 6)),
                       [sva.PTransformd(e3.Vector3d(0.,i,0.)) for i in range(7)])


    def test(v, func):
      pickled = pickle.dumps(v)
      v2 = pickle.loads(pickled)
      assert(func(v, v2))

    def bodyEq(b1, b2):
      return b1.inertia() == b2.inertia() and\
        b1.name() == b2.name()

    def jointEq(j1, j2):
      return j1.type() == j2.type() and\
        j1.name() == j2.name() and\
        j1.direction() == j2.direction() and\
        list(j1.motionSubspace()) == list(j2.motionSubspace())

    def multiBodyEq(mb1, mb2):
      isEq = True
      for b1, b2 in zip(mb1.bodies(), mb2.bodies()):
        isEq &= bodyEq(b1, b2)
      for j1, j2 in zip(mb1.joints(), mb2.joints()):
        isEq &= jointEq(j1, j2)

      mb1T = (list(mb1.predecessors()), list(mb1.successors()),
              list(mb1.parents()), list(mb1.transforms()))
      mb2T = (list(mb2.predecessors()), list(mb2.successors()),
              list(mb2.parents()), list(mb2.transforms()))
      return isEq and mb1T == mb2T


    # body
    test(body, bodyEq)

    # joints
    test(jR, jointEq)
    test(jP, jointEq)
    test(jS, jointEq)
    test(jPla, jointEq)
    test(jC, jointEq)
    test(jFree, jointEq)
    test(jFix, jointEq)

    # multiBody
    test(mb, multiBodyEq)
