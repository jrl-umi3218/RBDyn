# Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
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

from _rbdyn import *
import eigen3 as e3

# pickle, cPickle and copy support for Body, Joint and MultiBody

def bodyConstructor(inertia, bName):
  return Body(inertia, bName)

def bodyPickle(b):
  return bodyConstructor, (b.inertia(), b.name())


def jointConstructor(jType, axis, forward, jName):
  return Joint(jType, e3.Vector3d(*axis), forward, jName)

def jointPickle(j):
  axis = None

  # apply direction to reverse the motion vector on some joints
  def reverse(v):
    return j.direction()*v

  if j.type() == Joint.Rev:
    axis = e3.Vector3d(*map(reverse, list(j.motionSubspace())[:3]))
  elif j.type() == Joint.Prism:
    axis = e3.Vector3d(*map(reverse, list(j.motionSubspace())[3:]))
  elif j.type() == Joint.Spherical:
    axis = e3.Vector3d.UnitZ()
  elif j.type() == Joint.Planar:
    axis = e3.Vector3d.UnitZ()
  elif j.type() == Joint.Cylindrical:
    axis = e3.Vector3d(*map(reverse, list(j.motionSubspace())[:3]))
  elif j.type() == Joint.Free:
    axis = e3.Vector3d.UnitZ()
  elif j.type() == Joint.Fixed:
    axis = e3.Vector3d.UnitZ()
  else:
    raise RuntimeError('%s is an unknow joint type' % j.type())

  return jointConstructor, (j.type(), list(axis), j.forward(), j.name())


def multiBodyConstructor(bodies, joints, pred, succ, parent, Xt):
  return MultiBody(bodies, joints, pred, succ, parent, Xt)

def multiBodyPickle(mb):
  return multiBodyConstructor, (list(mb.bodies()), list(mb.joints()),
                                list(mb.predecessors()), list(mb.successors()),
                                list(mb.parents()), list(mb.transforms()))

def copy_reg_pickle():
  # python 2 and 3 support
  # first try to import copyreg (python 3)
  # if the import fail we import copy_reg (python 2)
  try:
    import copyreg
  except ImportError:
    import copy_reg as copyreg

  # register sva pickle needed by some rbdyn type
  import spacevecalg as sva
  sva.copy_reg_pickle()

  copyreg.pickle(Body, bodyPickle)
  copyreg.pickle(Joint, jointPickle)
  copyreg.pickle(MultiBody, multiBodyPickle)
