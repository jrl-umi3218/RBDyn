# distutils: language = c++

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

cimport rbdyn.c_rbdyn as c_rbdyn
cimport rbdyn.c_rbdyn_private as c_rbdyn_private
cimport sva.c_sva as c_sva
cimport eigen.c_eigen as c_eigen
cimport sva.sva as sva
cimport eigen.eigen as eigen

from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class DoubleVectorWrapper(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __cinit__(self, owner = None, *args):
    self.__own_impl = False
    self.__owner = owner
    pass
  def __getitem__(self, idx):
    if isinstance(idx, slice):
      return self.v[idx]
    else:
      if idx == -1:
        idx = self.v.size() - 1
      return self.v.at(idx)
  def __setitem__(self, idx, value):
    if isinstance(idx, slice):
      [start, stop, step] = idx.start, idx.stop, idx.step
      if start is None:
        start = 0
      if stop is None:
        stop = self.v.size()
      if step is None:
        step = 1
      j = 0
      for i in xrange(start, stop, step):
        c_rbdyn_private.dv_set_item(deref(self.v), i, value[j])
        j += 1
    else:
      c_rbdyn_private.dv_set_item(deref(self.v), idx, value)
  def __repr__(self):
    return repr(deref(self.v))
  def __copy__(self):
    cdef DoubleVectorWrapper ret = DoubleVectorWrapper()
    ret.__own_impl = True
    ret.v = new vector[double](deref(self.v))
    return ret
  def __deepcopy__(self, memo):
    cdef DoubleVectorWrapper ret = DoubleVectorWrapper()
    ret.__own_impl = True
    ret.v = new vector[double](deref(self.v))
    return ret
cdef DoubleVectorWrapper DoubleVectorWrapperFromC(vector[double] & v_in, owner):
  cdef DoubleVectorWrapper ret = DoubleVectorWrapper(owner)
  ret.v = &v_in
  return ret

cdef class DoubleVectorVectorWrapper(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __cinit__(self, owner = None, *args):
    self.__own_impl = False
    self.__owner = owner
    pass
  def __getitem__(self, idx):
    if isinstance(idx, slice):
      [start, stop, step] = idx.start, idx.stop, idx.step
      if start is None:
        start = 0
      if stop is None:
        stop = self.v.size()
      if step is None:
        step = 1
      ret = []
      for i in xrange(start, stop, step):
        ret.append(DoubleVectorWrapperFromC(self.v.at(i), self))
      return ret
    else:
      if idx == -1:
        idx = self.v.size() - 1
      return DoubleVectorWrapperFromC(self.v.at(idx), self)
  def __setitem__(self, idx, value):
    if isinstance(idx, slice):
      [start, stop, step] = idx.start, idx.stop, idx.step
      if start is None:
        start = 0
      if stop is None:
        stop = self.v.size()
      if step is None:
        step = 1
      j = 0
      for i in xrange(start, stop, step):
        c_rbdyn_private.dvv_set_item(deref(self.v), i, value[j])
        j += 1
    else:
      c_rbdyn_private.dvv_set_item(deref(self.v), idx, value)
  def __repr__(self):
    return repr(deref(self.v))
  def __copy__(self):
    cdef DoubleVectorVectorWrapper ret = DoubleVectorVectorWrapper()
    ret.__own_impl = True
    ret.v = new vector[vector[double]](deref(self.v))
    return ret
  def __deepcopy__(self, memo):
    cdef DoubleVectorVectorWrapper ret = DoubleVectorVectorWrapper()
    ret.__own_impl = True
    ret.v = new vector[vector[double]](deref(self.v))
    return ret
cdef DoubleVectorVectorWrapper DoubleVectorVectorWrapperFromC(vector[vector[double]] & v_in, owner):
  cdef DoubleVectorVectorWrapper ret = DoubleVectorVectorWrapper(owner)
  ret.v = &v_in
  return ret

cdef class Body(object):
  def __copyctor__(self, Body other):
    self.impl = other.impl
  def __rbictor__(self, sva.RBInertiad rbInertia, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl = c_rbdyn.Body(deref(rbInertia.impl), name)
  def __comictor__(self, double mass, eigen.Vector3d com, eigen.Matrix3d inertia, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl = c_rbdyn.Body(mass, com.impl, inertia.impl, name)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.Body()
    elif len(args) == 1 and isinstance(args[0], Body):
      self.__copyctor__(args[0])
    elif len(args) == 2 and isinstance(args[0], sva.RBInertiad):
      self.__rbictor__(args[0], args[1])
    elif len(args) == 4 and isinstance(args[1], eigen.Vector3d) and isinstance(args[2], eigen.Matrix3d):
      self.__comictor__(args[0], args[1], args[2], args[3])
    else:
      raise TypeError("Invalid arguments passed to Body ctor")

  def name(self):
    return self.impl.name()
  def inertia(self):
    return sva.RBInertiadFromC(self.impl.inertia())

  def __richcmp__(Body self, Body other, int op):
    if op == 2:
      return self.impl == other.impl
    elif op == 3:
      return self.impl != other.impl
    else:
      raise NotImplementedError("This comparison is not supported")

  def __str__(self):
    return c_rbdyn_private.BodyToString(self.impl).decode('utf-8')
  def __repr__(self):
    return self.__str__()

  @staticmethod
  def pickle(b):
    return Body, (b.inertia(), b.name())

cdef Body BodyFromC(const c_rbdyn.Body & b):
  cdef Body ret = Body()
  ret.impl = b
  return ret

cdef class BodyVector(object):
  def __addBody(self, Body b):
    self.v.push_back(b.impl)
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for b in args[0]:
        self.__addBody(b)
    elif len(args) == 1 and isinstance(args[0], Body):
      self.__addBody(args[0])
    else:
      for b in args[0]:
        self.__addBody(b)

cdef class Joint(object):
  Rev = c_rbdyn.JointRev
  Prism = c_rbdyn.JointPrism
  Spherical = c_rbdyn.JointSpherical
  Planar = c_rbdyn.JointPlanar
  Cylindrical = c_rbdyn.JointCylindrical
  Free = c_rbdyn.JointFree
  Fixed = c_rbdyn.JointFixed
  def __copyctor__(self, Joint other):
    self.impl = other.impl
  def __axisctor__(self, int t, eigen.Vector3d axis, cppbool forward, name):
    assert(t >= Joint.Rev and t <= Joint.Fixed)
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl = c_rbdyn.Joint(<c_rbdyn.JointType>t, axis.impl, forward, name)
  def __ctor__(self, int t, cppbool forward, name):
    assert(t >= Joint.Rev and t <= Joint.Fixed)
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl = c_rbdyn.Joint(<c_rbdyn.JointType>t, forward, name)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.Joint()
    elif len(args) == 1 and isinstance(args[0], Joint):
      self.__copyctor__(args[0])
    elif len(args) == 4 and isinstance(args[1], eigen.Vector3d):
      self.__axisctor__(args[0],args[1],args[2],args[3])
    elif len(args) == 4 and isinstance(args[1], list):
      self.__axisctor__(args[0],eigen.Vector3d(args[1]),args[2],args[3])
    elif len(args) == 3:
      self.__ctor__(args[0],args[1],args[2])
    else:
      raise TypeError("Invalid arguments passed to Joint ctor")

  def type(self):
    return self.impl.type()
  def direction(self):
    return self.impl.direction()
  def __forward(self, double f):
    self.impl.forward(f)
  def forward(self, f = None):
    if f is None:
      return self.impl.forward()
    else:
      self.__forward(f)
  def params(self):
    return self.impl.params()
  def dof(self):
    return self.impl.dof()
  def name(self):
    return self.impl.name()

  def motionSubspace(self):
    cdef eigen.MatrixXd ret = eigen.MatrixXd()
    ret.impl = self.impl.motionSubspace()
    return ret

  def pose(self, q):
    return sva.PTransformdFromC(self.impl.sPose(q))

  def motion(self, alpha):
    return sva.MotionVecdFromC(self.impl.sMotion(alpha))

  def zeroParam(self):
    return self.impl.zeroParam()
  def zeroDof(self):
    return self.impl.zeroDof()

  def __richcmp__(Joint self, Joint other, int op):
    if op == 2:
      return self.impl == other.impl
    elif op == 3:
      return self.impl != other.impl
    else:
      raise NotImplementedError("This comparison is not supported")

  def __str__(self):
    return c_rbdyn_private.JointToString(self.impl).decode('utf-8')
  def __repr__(self):
    return self.__str__()

  @staticmethod
  def ZeroParam(int jt):
    return c_rbdyn_private.ZeroParam(<c_rbdyn.JointType>jt)
  @staticmethod
  def ZeroDof(int jt):
    return c_rbdyn_private.ZeroDof(<c_rbdyn.JointType>jt)
  @staticmethod
  def pickle(j):
    axis = None
    def reverse(v):
      return j.direction() * v[0]
    if j.type() == Joint.Rev:
      axis = eigen.Vector3d(*map(reverse, list(j.motionSubspace())[:3]))
    elif j.type() == Joint.Prism:
      axis = eigen.Vector3d(*map(reverse, list(j.motionSubspace())[3:]))
    elif j.type() == Joint.Spherical:
      axis = eigen.Vector3d.UnitZ()
    elif j.type() == Joint.Planar:
      axis = eigen.Vector3d.UnitZ()
    elif j.type() == Joint.Cylindrical:
      axis = eigen.Vector3d(*map(reverse, list(j.motionSubspace())[:3]))
    elif j.type() == Joint.Free:
      axis = eigen.Vector3d.UnitZ()
    elif j.type() == Joint.Fixed:
      axis = eigen.Vector3d.UnitZ()
    else:
      raise RuntimeError('%s is an unknow joint type' % j.type())
    return Joint, (j.type(), list(axis), j.forward(), j.name())

cdef Joint JointFromC(const c_rbdyn.Joint & j):
  cdef Joint ret = Joint()
  ret.impl = j
  return ret

cdef class JointVector(object):
  def __addJoint(self, Joint j):
    self.v.push_back(j.impl)
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for j in args[0]:
        self.__addJoint(j)
    elif len(args) == 1 and isinstance(args[0], Joint):
        self.__addJoint(args[0])
    else:
        for j in args:
          self.__addJoint(j)

cdef class MultiBody(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __copyctor__(self, MultiBody other):
    self.impl = new c_rbdyn.MultiBody(deref(other.impl))
  def __ctor__(self, BodyVector bodies, JointVector joints, pred, succ, parent, sva.PTransformdVector Xt):
    self.impl = new c_rbdyn.MultiBody(bodies.v, joints.v, pred, succ, parent, deref(Xt.v))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if len(args) == 0:
      if not skip_alloc:
        self.impl = new c_rbdyn.MultiBody()
    elif len(args) == 1 and isinstance(args[0], MultiBody):
      self.__copyctor__(args[0])
    elif len(args) == 6:
      self.__ctor__(BodyVector(args[0]), JointVector(args[1]), args[2], args[3], args[4], sva.PTransformdVector(args[5]))
    else:
      raise TypeError("Invalid arguments passed to MultiBody ctor")
  def nrBodies(self):
    return self.impl.nrBodies()
  def nrJoints(self):
    return self.impl.nrJoints()
  def nrParams(self):
    return self.impl.nrParams()
  def nrDof(self):
    return self.impl.nrDof()
  def __getBodies(self):
    ret = []
    cdef vector[c_rbdyn.Body] bodies = self.impl.bodies()
    for b in bodies:
      ret.append(BodyFromC(b))
    return ret
  def __setBodies(self, BodyVector bodies):
    self.impl.sBodies(bodies.v)
  def bodies(self, bodies = None):
    if bodies is None:
      return self.__getBodies()
    else:
      self.__setBodies(BodyVector(bodies))
  def body(self, int i, Body b = None):
    if b is None:
      return BodyFromC(self.impl.sBody(i))
    else:
      self.impl.sBody(i, b.impl)
  def joints(self):
    ret = []
    cdef vector[c_rbdyn.Joint] joints = self.impl.joints()
    for j in joints:
      ret.append(JointFromC(j))
    return ret
  def joint(self, int i):
    return JointFromC(self.impl.sJoint(i))
  def predecessors(self):
    return self.impl.predecessors()
  def predecessor(self, int i):
    return self.impl.sPredecessor(i)
  def successors(self):
    return self.impl.successors()
  def successor(self, int i):
    return self.impl.sSuccessor(i)
  def parents(self):
    return self.impl.parents()
  def parent(self, int i):
    return self.impl.sParent(i)
  def __setTransforms(self, sva.PTransformdVector tfs):
    self.impl.sTransforms(deref(tfs.v))
  def __getTransforms(self):
    ret = []
    cdef vector[c_sva.PTransformd] tfs = self.impl.transforms()
    for tf in tfs:
      ret.append(sva.PTransformdFromC(tf))
    return ret
  def transforms(self, tfs = None):
    if tfs is None:
      return self.__getTransforms()
    else:
      self.__setTransforms(sva.PTransformdVector(tfs))
  def transform(self, int i, sva.PTransformd tf = None):
    if tf is None:
      return sva.PTransformdFromC(self.impl.sTransform(i))
    else:
      self.impl.sTransform(i, deref(tf.impl))
  def jointsPosInParam(self):
    return self.impl.jointsPosInParam()
  def jointPosInParam(self, int i):
    return self.impl.sJointPosInParam(i)
  def jointsPosInDof(self):
    return self.impl.jointsPosInDof()
  def jointPosInDof(self, int i):
    return self.impl.sJointPosInDof(i)
  def bodyIndexByName(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.sBodyIndexByName(name)
  def jointIndexByName(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.sJointIndexByName(name)
  @staticmethod
  def pickle(mb):
    return MultiBody, (list(mb.bodies()), list(mb.joints()),
                       list(mb.predecessors()), list(mb.successors()),
                       list(mb.parents()), list(mb.transforms()))

cdef class MultiBodyVector(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __addMultiBody(self, MultiBody j):
    self.v.push_back(deref(j.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.v = new vector[c_rbdyn.MultiBody]()
      if len(args) == 1 and isinstance(args[0], list):
        for j in args[0]:
          self.__addMultiBody(j)
      else:
          for j in args:
            self.__addMultiBody(j)
  def __getitem__(self, int idx):
    if idx == -1:
      idx = self.v.size() - 1
    if idx < self.v.size():
      return MultiBodyFromC(self.v.at(idx), False)
    else:
      raise IndexError

cdef MultiBodyVector MultiBodyVectorFromPtr(vector[c_rbdyn.MultiBody]* p):
  cdef MultiBodyVector ret = MultiBodyVector(skip_alloc = True)
  ret.__own_impl = False
  ret.v = p
  return ret

cdef class MultiBodyGraph(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __copyctor__(self, MultiBodyGraph other):
    self.impl = new c_rbdyn.MultiBodyGraph(deref(other.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if len(args) == 0:
      if not skip_alloc:
        self.impl = new c_rbdyn.MultiBodyGraph()
    elif len(args) == 1 and isinstance(args[0], MultiBodyGraph):
      self.__copyctor__(args[0])
    else:
      raise TypeError("Invalid arguments passed to MultiBodyGraph ctor")
  def addBody(self, Body b):
    self.impl.addBody(b.impl)
  def addJoint(self, Joint j):
    self.impl.addJoint(j.impl)
  def linkBodies(self, b1Name, sva.PTransformd tB1, b2Name, sva.PTransformd tB2, jointName, cppbool isB1toB2 = True):
    if isinstance(b1Name, unicode):
      b1Name = b1Name.encode('ascii')
    if isinstance(b2Name, unicode):
      b2Name = b2Name.encode('ascii')
    if isinstance(jointName, unicode):
      jointName = jointName.encode('ascii')
    self.impl.linkBodies(b1Name, deref(tB1.impl), b2Name, deref(tB2.impl), jointName, isB1toB2)
  def nrNodes(self):
    return self.impl.nrNodes()
  def nrJoints(self):
    return self.impl.nrJoints()
  def removeJoint(self, rootBodyName, arg):
      self.impl.removeJoint(rootBodyName, arg)
  def removeJoints(self, rootBodyName, arg):
    if isinstance(arg, list) and len(arg) > 0:
      if isinstance(arg[0], basestring):
        self.impl.removeJointsSV(rootBodyName,arg)
  def mergeSubBodies(self, rootBodyName, jName, arg):
    if isinstance(arg, dict) and len(arg) > 0 and isinstance(arg.values()[0],list):
        self.impl.mergeSubBodies(rootBodyName, jName, <cppmap[string,vector[double]]>arg)
  def __makeMB(self, rBName, cppbool isFixed, sva.PTransformd X_0_j0 = sva.PTransformd.Identity(), sva.PTransformd X_b0_j0 = sva.PTransformd.Identity()):
    return MultiBodyFromC(self.impl.makeMultiBody(rBName, isFixed,
      deref(X_0_j0.impl), deref(X_b0_j0.impl)))
  def __makeMBWAxis(self, rBName, c_rbdyn.JointType tp, eigen.Vector3d ax, sva.PTransformd X_0_j0 = sva.PTransformd.Identity(), sva.PTransformd X_b0_j0 = sva.PTransformd.Identity()):
    return MultiBodyFromC(self.impl.makeMultiBody(rBName, tp, ax.impl,
      deref(X_0_j0.impl), deref(X_b0_j0.impl)))
  def makeMultiBody(self, rootBodyName, *args):
    if isinstance(rootBodyName, unicode):
      rootBodyName = rootBodyName.encode(u'ascii')
    # Possible calls
    # bool, (PT, (PT))
    # int, V3d, (PT, (PT))
    if len(args) > 0:
      if isinstance(args[0], bool):
        if len(args) > 1 and isinstance(args[1], sva.PTransformd):
          if len(args) == 3  and isinstance(args[2], sva.PTransformd):
            return self.__makeMB(rootBodyName, args[0], args[1], args[2])
          elif len(args) == 2:
            return self.__makeMB(rootBodyName, args[0], args[1])
        elif len(args) == 1:
          return self.__makeMB(rootBodyName, args[0])
      elif len(args) > 1 and isinstance(args[1], eigen.Vector3d):
        if len(args) > 2 and isinstance(args[2], sva.PTransformd):
          if len(args) == 4  and isinstance(args[3], sva.PTransformd):
            return self.__makeMBWAxis(rootBodyName, args[0], args[1], args[2], args[3])
          elif len(args) == 3:
            return self.__makeMBWAxis(rootBodyName, args[0], args[1], args[2])
        elif len(args) == 2:
          return self.__makeMBWAxis(rootBodyName, args[0], args[1])
    else:
      raise TypeError("Not enough arguments passed to makeMultiBody")
    raise TypeError("Wrong arguments passed to makeMultiBody")
  def bodiesBaseTransform(self, rootBodyName, sva.PTransformd X_b0_j0 = sva.PTransformd.Identity()):
    cdef cppmap[string,c_sva.PTransformd] res = self.impl.bodiesBaseTransform(rootBodyName, deref(X_b0_j0.impl))
    ret = {}
    for it in res:
      ret[it.first] = sva.PTransformdFromC(it.second)
    return ret

  def successorJoints(self, rootBodyName):
    cdef cppmap[string, vector[string]] res = self.impl.successorJoints(rootBodyName)
    return res

cdef MultiBody MultiBodyFromC(const c_rbdyn.MultiBody& mb, cppbool copy = True):
  cdef MultiBody ret = MultiBody(skip_alloc = True)
  if copy:
    ret.impl = new c_rbdyn.MultiBody(mb)
  else:
    ret.__own_impl = False
    ret.impl = &(c_rbdyn_private.const_cast_mb(mb))
  return ret

cdef MultiBodyGraph MultiBodyGraphFromC(const c_rbdyn.MultiBodyGraph & mbg, cppbool copy = True):
  cdef MultiBodyGraph ret = MultiBodyGraph(skip_alloc = True)
  if copy:
    ret.impl = new c_rbdyn.MultiBodyGraph(mbg)
  else:
    ret.__own_impl = False
    ret.impl = &(c_rbdyn_private.const_cast_mbg(mbg))
  return ret

cdef class MotionSubspaceVector(object):
  cdef vector[c_eigen.Matrix[double,c_eigen.six,c_eigen.dynamic]] v
  def __addMatrix(self, eigen.MatrixXd m):
    self.v.push_back(<c_eigen.Matrix[double,c_eigen.six,c_eigen.dynamic]>(m.impl))
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for m in args[0]:
        self.__addMatrix(m)
    elif len(args) == 1 and isinstance(args[0], eigen.MatrixXd):
      self.__addMatrix(args[0])
    else:
      for m in args:
        self.__addMatrix(m)

cdef class MultiBodyConfig(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __copyctor__(self, MultiBodyConfig other):
    self.impl = new c_rbdyn.MultiBodyConfig(deref(other.impl))
  def __mbctor__(self, MultiBody other):
    self.impl = new c_rbdyn.MultiBodyConfig(deref(other.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if len(args) == 0:
      if not skip_alloc:
        self.impl = new c_rbdyn.MultiBodyConfig()
    elif len(args) == 1 and isinstance(args[0], MultiBodyConfig):
      self.__copyctor__(args[0])
    elif len(args) == 1 and isinstance(args[0], MultiBody):
      self.__mbctor__(args[0])
    else:
      raise TypeError("Invalid arguments passed to MultiBodyConfig ctor")
  def zero(self, MultiBody mb):
    self.impl.zero(deref(mb.impl))
  property q:
    def __get__(self):
      return DoubleVectorVectorWrapperFromC(self.impl.q, self)
    def __set__(self, value):
      self.impl.q = value
  property alpha:
    def __get__(self):
      return DoubleVectorVectorWrapperFromC(self.impl.alpha, self)
    def __set__(self, value):
      self.impl.alpha = value
  property alphaD:
    def __get__(self):
      return DoubleVectorVectorWrapperFromC(self.impl.alphaD, self)
    def __set__(self, value):
      self.impl.alphaD = value
  property force:
    def __get__(self):
      ret = []
      for fv in self.impl.force:
        ret.append(sva.ForceVecdFromC(fv, copy = False))
      return ret
    def __set__(self, value):
      self.impl.force = sva.ForceVecdVector(value).v
  property jointConfig:
    def __get__(self):
      ret = []
      for jc in self.impl.jointConfig:
        ret.append(sva.PTransformdFromC(jc))
      return ret
    def __set__(self, value):
      self.impl.jointConfig = deref(sva.PTransformdVector(value).v)
  property jointVelocity:
    def __get__(self):
      ret = []
      for jv in self.impl.jointVelocity:
        ret.append(sva.MotionVecdFromC(jv))
      return ret
    def __set__(self, value):
      self.impl.jointVelocity = sva.MotionVecdVector(value).v
  property jointTorque:
    def __get__(self):
      return self.impl.jointTorque
    def __set__(self, value):
      self.impl.jointTorque = value
  property motionSubspace:
    def __get__(self):
      ret = []
      for m in self.impl.motionSubspace:
        # Cast needed here because it's Eigen::Matrix<double,6,-1>
        ret.append(eigen.MatrixXdFromC(<c_eigen.MatrixXd>m))
      return ret
    def __set__(self, value):
      self.impl.motionSubspace = MotionSubspaceVector(value).v
  property bodyPosW:
    def __get__(self):
      ret = []
      for pt in self.impl.bodyPosW:
        ret.append(sva.PTransformdFromC(pt))
      return ret
    def __set__(self, value):
      self.impl.bodyPosW = deref(sva.PTransformdVector(value).v)
  property parentToSon:
    def __get__(self):
      ret = []
      for pt in self.impl.parentToSon:
        ret.append(sva.PTransformdFromC(pt))
      return ret
    def __set__(self, value):
      self.impl.parentToSon = deref(sva.PTransformdVector(value).v)
  property bodyVelW:
    def __get__(self):
      ret = []
      for jv in self.impl.bodyVelW:
        ret.append(sva.MotionVecdFromC(jv))
      return ret
    def __set__(self, value):
      self.impl.bodyVelW = sva.MotionVecdVector(value).v
  property bodyVelB:
    def __get__(self):
      ret = []
      for jv in self.impl.bodyVelB:
        ret.append(sva.MotionVecdFromC(jv))
      return ret
    def __set__(self, value):
      self.impl.bodyVelB = sva.MotionVecdVector(value).v
  property bodyAccB:
    def __get__(self):
      ret = []
      for jv in self.impl.bodyAccB:
        ret.append(sva.MotionVecdFromC(jv))
      return ret
    def __set__(self, value):
      self.impl.bodyAccB = sva.MotionVecdVector(value).v
  property gravity:
    def __get__(self):
      return eigen.Vector3dFromC(self.impl.gravity)
    def __set__(self, eigen.Vector3d v):
      self.impl.gravity = v.impl

cdef MultiBodyConfig MultiBodyConfigFromC(const c_rbdyn.MultiBodyConfig& mbc, cppbool copy = True):
  cdef MultiBodyConfig ret = MultiBodyConfig(skip_alloc = True)
  if copy:
    ret.impl = new c_rbdyn.MultiBodyConfig(mbc)
  else:
    ret.__own_impl = False
    ret.impl = &(c_rbdyn_private.const_cast_mbc(mbc))
  return ret

cdef class MultiBodyConfigVector(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __addMultiBodyConfig(self, MultiBodyConfig j):
    self.v.push_back(deref(j.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.v = new vector[c_rbdyn.MultiBodyConfig]()
      if len(args) == 1 and isinstance(args[0], list):
        for j in args[0]:
          self.__addMultiBodyConfig(j)
      else:
          for j in args:
            self.__addMultiBodyConfig(j)
  def __getitem__(self, int idx):
    if idx == -1:
      idx = self.v.size() - 1
    if idx < self.v.size():
      return MultiBodyConfigFromC(self.v.at(idx), False)
    else:
      raise IndexError
  def __setitem__(self, int idx, MultiBodyConfig mbc):
    if idx < self.v.size():
      c_rbdyn_private.mbcv_set_item(deref(self.v), idx, deref(mbc.impl))
    else:
      raise IndexError

cdef MultiBodyConfigVector MultiBodyConfigVectorFromPtr(vector[c_rbdyn.MultiBodyConfig]* p):
  cdef MultiBodyConfigVector ret = MultiBodyConfigVector(skip_alloc = True)
  ret.__own_impl = False
  ret.v = p
  return ret

cdef class ConfigConverter(object):
  def __copyctor__(self, ConfigConverter other):
    self.impl = new c_rbdyn.ConfigConverter(deref(other.impl))
  def __mbsctor__(self, MultiBody mb1, MultiBody mb2):
    self.impl = c_rbdyn_private.ConfigConverterConstructor(deref(mb1.impl),
        deref(mb2.impl))
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], ConfigConverter):
      self.__copyctor__(args[0])
    elif len(args) == 2 and isinstance(args[0], MultiBody) and isinstance(args[1], MultiBody):
      self.__mbsctor__(args[0], args[1])
    else:
      raise TypeError("Invalid arguments passed to ConfigConverter ctor")
  def __dealloc__(self):
    del self.impl
  def convert(self, MultiBodyConfig mbc1, MultiBodyConfig mbc2):
    self.impl.sConvert(deref(mbc1.impl), deref(mbc2.impl))
  def convertJoint(self, _from):
    if len(_from):
      if isinstance(_from[0], list):
        self.impl.convertJointVV(_from)
      else:
        self.impl.convertJointV(_from)
    else:
      raise TypeError("convertJoint called with wrong arguments")

def __paramToVectorMB(MultiBody mb, q):
  return eigen.VectorXdFromC(c_rbdyn.rSParamToVector(deref(mb.impl), q))

def __paramToVectorVXd(q, eigen.VectorXd v):
  c_rbdyn.sParamToVector(q, v.impl)

def paramToVector(arg1, arg2):
  if isinstance(arg1, MultiBody):
    return __paramToVectorMB(arg1, arg2)
  elif isinstance(arg2, eigen.VectorXd):
    __paramToVectorVXd(arg1, arg2)
  else:
    raise TypeError("Wrong types passed to paramToVector")

def __vectorToParamMB(MultiBody mb, eigen.VectorXd v):
  return c_rbdyn.rSVectorToParam(deref(mb.impl), v.impl)

def __vectorToParamV(eigen.VectorXd v, q):
  if isinstance(q, DoubleVectorVectorWrapper):
    c_rbdyn.sVectorToParam(v.impl, deref((<DoubleVectorVectorWrapper>(q)).v))
    return
  cdef vector[vector[double]] qv = q
  c_rbdyn.sVectorToParam(v.impl, qv)
  return qv

def vectorToParam(arg1, arg2):
  if isinstance(arg1, MultiBody) and isinstance(arg2, eigen.VectorXd):
    return __vectorToParamMB(arg1, arg2)
  elif isinstance(arg1, eigen.VectorXd):
    return __vectorToParamV(arg1, arg2)
  else:
    return __vectorToParamV(eigen.VectorXd(arg1), arg2)

def dofToVector(MultiBody mb, q):
  return eigen.VectorXdFromC(c_rbdyn.sDofToVector(deref(mb.impl), q))

def vectorToDof(MultiBody mb, eigen.VectorXd v):
  return c_rbdyn.sVectorToDof(deref(mb.impl), v.impl)

cdef class Jacobian(object):
  def __copyctor__(self, Jacobian other):
    self.impl = c_rbdyn.Jacobian(other.impl)
  def __mbvctor__(self, MultiBody mb, bodyName, eigen.Vector3d point = eigen.Vector3d.Zero()):
    self.impl = c_rbdyn.Jacobian(deref(mb.impl), bodyName, point.impl)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.Jacobian()
    elif len(args) == 1 and isinstance(args[0], Jacobian):
      self.__copyctor__(args[0])
    elif len(args) >= 2 and isinstance(args[0], MultiBody):
      if len(args) == 3 and isinstance(args[2], eigen.Vector3d):
        self.__mbvctor__(args[0], args[1], args[2])
      elif len(args) == 2:
        self.__mbvctor__(args[0], args[1])
    else:
      raise TypeError("Invalid arguments passed to Jacobian ctor")
  def jacobian(self, MultiBody mb, MultiBodyConfig mbc, sva.PTransformd X_0_p = None):
    if X_0_p is None:
      return eigen.MatrixXdFromC(self.impl.sJacobian(deref(mb.impl), deref(mbc.impl)))
    else:
      return eigen.MatrixXdFromC(self.impl.sJacobian(deref(mb.impl),
        deref(mbc.impl), deref(X_0_p.impl)))
  def jacobianDot(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sJacobianDot(deref(mb.impl), deref(mbc.impl)))
  def bodyJacobian(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sBodyJacobian(deref(mb.impl), deref(mbc.impl)))
  def bodyJacobianDot(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sBodyJacobianDot(deref(mb.impl), deref(mbc.impl)))
  def vectorJacobian(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d v):
    return eigen.MatrixXdFromC(self.impl.sVectorJacobian(deref(mb.impl), deref(mbc.impl), v.impl))
  def vectorBodyJacobian(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d v):
    return eigen.MatrixXdFromC(self.impl.sVectorBodyJacobian(deref(mb.impl), deref(mbc.impl), v.impl))
  def translateJacobian(self, eigen.MatrixXd jac, MultiBodyConfig mbc, eigen.Vector3d point, eigen.MatrixXd res):
    self.impl.sTranslateJacobian(jac.impl, deref(mbc.impl), point.impl, res.impl)
  def fullJacobian(self, MultiBody mb, eigen.MatrixXd jac, eigen.MatrixXd res):
    self.impl.sFullJacobian(deref(mb.impl), jac.impl, res.impl)
  def subMultiBody(self, MultiBody mb):
    return MultiBodyFromC(self.impl.sSubMultiBody(deref(mb.impl)))
  def jointsPath(self):
    return self.impl.jointsPath()
  def dof(self):
    return self.impl.dof()
  def point(self, eigen.Vector3d point = None):
    if point is None:
      return eigen.Vector3dFromC(self.impl.point())
    else:
      self.impl.point(point.impl)
  def velocity(self, MultiBody mb, MultiBodyConfig mbc, sva.PTransformd X_b_p = None):
    if X_b_p is None:
      return sva.MotionVecdFromC(self.impl.sVelocity(deref(mb.impl), deref(mbc.impl)))
    else:
      return sva.MotionVecdFromC(self.impl.sVelocity(deref(mb.impl),
        deref(mbc.impl), deref(X_b_p.impl)))
  def bodyVelocity(self, MultiBody mb, MultiBodyConfig mbc):
    return sva.MotionVecdFromC(self.impl.sBodyVelocity(deref(mb.impl), deref(mbc.impl)))
  def __normalAccelerationPTMV(self, MultiBody mb, MultiBodyConfig mbc, sva.PTransformd X_b_p, sva.MotionVecd V_b_p):
    return sva.MotionVecdFromC(self.impl.sNormalAcceleration(deref(mb.impl),
      deref(mbc.impl), deref(X_b_p.impl), deref(V_b_p.impl)))
  def __normalAccelerationAccPTMV(self, MultiBody mb, MultiBodyConfig mbc, normalAccB, sva.PTransformd X_b_p, sva.MotionVecd V_b_p):
    return sva.MotionVecdFromC(self.impl.sNormalAcceleration(deref(mb.impl),
      deref(mbc.impl), sva.MotionVecdVector(normalAccB).v, deref(X_b_p.impl),
      deref(V_b_p.impl)))
  def normalAcceleration(self, MultiBody mb, MultiBodyConfig mbc, *args):
    if len(args) == 0:
      return sva.MotionVecdFromC(self.impl.sNormalAcceleration(deref(mb.impl), deref(mbc.impl)))
    if len(args) == 1 and isinstance(args[0], list):
      return sva.MotionVecdFromC(self.impl.sNormalAcceleration(deref(mb.impl), deref(mbc.impl), sva.MotionVecdVector(args[0]).v))
    if len(args) == 2 and isinstance(args[0], sva.PTransformd) and isinstance(args[1], sva.MotionVecd):
      return self.__normalAccelerationPTMV(mb, mbc, args[0], args[1])
    if len(args) == 3 and isinstance(args[0], list) and isinstance(args[1], sva.PTransformd) and isinstance(args[2], sva.MotionVecd):
      return self.__normalAccelerationAccPTMV(mb, mbc, args[0], args[1], args[2])
    raise TypeError("Wrong arguments passed to normalAcceleration")
  def bodyNormalAcceleration(self, MultiBody mb, MultiBodyConfig mbc, normalAccB = None):
    if normalAccB is None:
      return sva.MotionVecdFromC(self.impl.sBodyNormalAcceleration(deref(mb.impl), deref(mbc.impl)))
    else:
      return sva.MotionVecdFromC(self.impl.sBodyNormalAcceleration(deref(mb.impl), deref(mbc.impl), sva.MotionVecdVector(normalAccB).v))

def forwardKinematics(MultiBody mb, MultiBodyConfig mbc):
  c_rbdyn.sForwardKinematics(deref(mb.impl), deref(mbc.impl))

def forwardVelocity(MultiBody mb, MultiBodyConfig mbc):
  c_rbdyn.sForwardVelocity(deref(mb.impl), deref(mbc.impl))

def forwardAcceleration(MultiBody mb, MultiBodyConfig mbc, sva.MotionVecd A_0 = sva.MotionVecd(eigen.Vector6d.Zero())):
  c_rbdyn.sForwardAcceleration(deref(mb.impl), deref(mbc.impl), deref(A_0.impl))

def eulerIntegration(MultiBody mb, MultiBodyConfig mbc, double step):
  c_rbdyn.sEulerIntegration(deref(mb.impl), deref(mbc.impl), step)

cdef class InverseDynamics(object):
  def __copyctor__(self, InverseDynamics other):
    self.impl = c_rbdyn.InverseDynamics(other.impl)
  def __mbctor__(self, MultiBody mb):
    self.impl = c_rbdyn.InverseDynamics(deref(mb.impl))
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.InverseDynamics()
    elif len(args) == 1:
      if isinstance(args[0], InverseDynamics):
        self.__copyctor__(args[0])
      elif isinstance(args[0], MultiBody):
        self.__mbctor__(args[0])
    else:
      raise TypeError("Invalid arguments passed to InverseDynamics ctor")
  def inverseDynamics(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sInverseDynamics(deref(mb.impl), deref(mbc.impl))
  def inverseDynamicsNoInertia(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sInverseDynamicsNoInertia(deref(mb.impl), deref(mbc.impl))
  def f(self):
    cdef vector[c_sva.ForceVecd] fvv = self.impl.f()
    ret = []
    for fv in fvv:
      ret.append(sva.ForceVecdFromC(fv))
    return ret

cdef class ForwardDynamics(object):
  def __copyctor__(self, ForwardDynamics other):
    self.impl = c_rbdyn.ForwardDynamics(other.impl)
  def __mbctor__(self, MultiBody mb):
    self.impl = c_rbdyn.ForwardDynamics(deref(mb.impl))
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.ForwardDynamics()
    elif len(args) == 1:
      if isinstance(args[0], ForwardDynamics):
        self.__copyctor__(args[0])
      elif isinstance(args[0], MultiBody):
        self.__mbctor__(args[0])
    else:
      raise TypeError("Invalid arguments passed to ForwardDynamics ctor")

  def forwardDynamics(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sForwardDynamics(deref(mb.impl), deref(mbc.impl))

  def computeH(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sComputeH(deref(mb.impl), deref(mbc.impl))

  def computeC(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sComputeC(deref(mb.impl), deref(mbc.impl))

  def H(self):
    return eigen.MatrixXdFromC(self.impl.H())

  def C(self):
    return eigen.VectorXdFromC(self.impl.C())

  def inertiaSubTree(self):
    cdef vector[c_sva.RBInertiad] rbiv = self.impl.inertiaSubTree()
    ret = []
    for rbi in rbiv:
      ret.append(sva.RBInertiadFromC(rbi))
    return ret

cdef class Coriolis(object):
  def __dealloc__(self):
    del self.impl
  def __cinit__(self, MultiBody mb):
    self.impl = new c_rbdyn.Coriolis(deref(mb.impl))
  def coriolis(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.coriolis(deref(mb.impl), deref(mbc.impl)))

def computeCoM(MultiBody mb, MultiBodyConfig mbc):
  return eigen.Vector3dFromC(c_rbdyn.sComputeCoM(deref(mb.impl), deref(mbc.impl)))

def computeCoMVelocity(MultiBody mb, MultiBodyConfig mbc):
  return eigen.Vector3dFromC(c_rbdyn.sComputeCoMVelocity(deref(mb.impl), deref(mbc.impl)))

def computeCoMAcceleration(MultiBody mb, MultiBodyConfig mbc):
  return eigen.Vector3dFromC(c_rbdyn.sComputeCoMAcceleration(deref(mb.impl), deref(mbc.impl)))

cdef class CoMJacobianDummy(object):
  def __copyctor__(self, CoMJacobianDummy other):
    self.impl = c_rbdyn.CoMJacobianDummy(other.impl)
  def __mbctor__(self, MultiBody mb, q = None):
    if q is None:
      self.impl = c_rbdyn.CoMJacobianDummy(deref(mb.impl))
    else:
      self.impl = c_rbdyn.CoMJacobianDummy(deref(mb.impl), q)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.CoMJacobianDummy()
    elif len(args) == 1 and isinstance(args[0], CoMJacobianDummy):
      self.__copyctor__(args[0])
    elif len(args) > 0 and isinstance(args[0], MultiBody):
      if len(args) == 1:
        self.__mbctor__(args[0])
      elif len(args) == 2 and isinstance(args[1], list):
        self.__mbctor__(args[0], args[1])
    else:
      raise TypeError("Invalid arguments passed to CoMJacobianDummy ctor")
  def jacobian(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sJacobian(deref(mb.impl), deref(mbc.impl)))
  def jacobianDot(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sJacobianDot(deref(mb.impl), deref(mbc.impl)))

cdef class CoMJacobian(object):
  def __copyctor__(self, CoMJacobian other):
    self.impl = c_rbdyn.CoMJacobian(other.impl)
  def __mbctor__(self, MultiBody mb, q = None):
    if q is None:
      self.impl = c_rbdyn.CoMJacobian(deref(mb.impl))
    else:
      self.impl = c_rbdyn.CoMJacobian(deref(mb.impl), q)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.CoMJacobian()
    elif len(args) == 1 and isinstance(args[0], CoMJacobian):
      self.__copyctor__(args[0])
    elif len(args) > 0 and isinstance(args[0], MultiBody):
      if len(args) == 1:
        self.__mbctor__(args[0])
      elif len(args) == 2 and isinstance(args[1], list):
        self.__mbctor__(args[0], args[1])
    else:
      raise TypeError("Invalid arguments passed to CoMJacobian ctor")
  def jacobian(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sJacobian(deref(mb.impl), deref(mbc.impl)))
  def jacobianDot(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.MatrixXdFromC(self.impl.sJacobianDot(deref(mb.impl), deref(mbc.impl)))
  def updateInertialParameters(self, MultiBody mb):
    self.impl.sUpdateInertialParameters(deref(mb.impl))
  def weight(self, MultiBody mb = None, w = None):
    if mb is None:
      return self.impl.weight()
    elif mb is not None and w is not None:
      self.impl.sWeight(deref(mb.impl), w)
    else:
      raise TypeError("You need to pass weights to set them")
  def velocity(self, MultiBody mb, MultiBodyConfig mbc):
    return eigen.Vector3dFromC(self.impl.sVelocity(deref(mb.impl), deref(mbc.impl)))
  def normalAcceleration(self, MultiBody mb, MultiBodyConfig mbc, mvecvec = None):
    if mvecvec is None:
      return eigen.Vector3dFromC(self.impl.sNormalAcceleration(deref(mb.impl), deref(mbc.impl)))
    else:
      return eigen.Vector3dFromC(self.impl.sNormalAcceleration(deref(mb.impl), deref(mbc.impl), sva.MotionVecdVector(mvecvec).v))

def computeCentroidalMomentum(MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com):
  return sva.ForceVecdFromC(c_rbdyn.sComputeCentroidalMomentum(deref(mb.impl), deref(mbc.impl), com.impl))
def computeCentroidalMomentumDot(MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com, eigen.Vector3d comVel):
  return sva.ForceVecdFromC(c_rbdyn.sComputeCentroidalMomentumDot(deref(mb.impl), deref(mbc.impl), com.impl, comVel.impl))

cdef class CentroidalMomentumMatrix(object):
  def __copyctor__(self, CentroidalMomentumMatrix other):
    self.impl = c_rbdyn.CentroidalMomentumMatrix(other.impl)
  def __mbctor__(self, MultiBody mb, weight = None):
    if weight is None:
      self.impl = c_rbdyn.CentroidalMomentumMatrix(deref(mb.impl))
    else:
      self.impl = c_rbdyn.CentroidalMomentumMatrix(deref(mb.impl), weight)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.impl = c_rbdyn.CentroidalMomentumMatrix()
    elif len(args) == 1 and isinstance(args[0], CentroidalMomentumMatrix):
      self.__copyctor__(args[0])
    elif len(args) > 0 and isinstance(args[0], MultiBody):
      if len(args) == 1:
        self.__mbctor__(args[0])
      elif len(args) == 2 and isinstance(args[1], list):
        self.__mbctor__(args[0], args[1])
    else:
      raise TypeError("Invalid arguments passed to CentroidalMomentumMatrix ctor")
  def computeMatrix(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com):
    self.impl.sComputeMatrix(deref(mb.impl), deref(mbc.impl), com.impl)
  def computeMatrixDot(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com, eigen.Vector3d comDot):
    self.impl.sComputeMatrixDot(deref(mb.impl), deref(mbc.impl), com.impl, comDot.impl)
  def computeMatrixAndMatrixDot(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com, eigen.Vector3d comDot):
    self.impl.sComputeMatrixAndMatrixDot(deref(mb.impl), deref(mbc.impl), com.impl, comDot.impl)

  def matrix(self):
    return eigen.MatrixXdFromC(self.impl.matrix())
  def matrixDot(self):
    return eigen.MatrixXdFromC(self.impl.matrixDot())

  def momentum(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com):
    return sva.ForceVecdFromC(self.impl.sMomentum(deref(mb.impl), deref(mbc.impl), com.impl))
  def normalMomentumDot(self, MultiBody mb, MultiBodyConfig mbc, eigen.Vector3d com, eigen.Vector3d comDot, normalAccB = None):
    if normalAccB is None:
      return sva.ForceVecdFromC(self.impl.sNormalMomentumDot(deref(mb.impl), deref(mbc.impl), com.impl, comDot.impl))
    else:
      return sva.ForceVecdFromC(self.impl.sNormalMomentumDot(deref(mb.impl), deref(mbc.impl), com.impl, comDot.impl, sva.MotionVecdVector(normalAccB).v))

def computeCentroidalZMP(MultiBodyConfig mbc, eigen.Vector3d com, eigen.Vector3d comA, double altitude):
  return eigen.Vector3dFromC(c_rbdyn.computeCentroidalZMP(deref(mbc.impl), com.impl, comA.impl, altitude))

def IMPhi(sva.MotionVecd mv):
  return eigen.MatrixXdFromC(<c_eigen.MatrixXd>c_rbdyn.IMPhi(deref(mv.impl)))

def inertiaToVector(sva.RBInertiad rbi):
  return eigen.VectorXdFromC(<c_eigen.VectorXd>c_rbdyn.inertiaToVector(deref(rbi.impl)))

def sVectorToInertia(eigen.VectorXd vec):
  return sva.RBInertiadFromC(c_rbdyn.sVectorToInertia(vec.impl))

def multiBodyToInertialVector(MultiBody mb):
  return eigen.VectorXdFromC(c_rbdyn.multiBodyToInertialVector(deref(mb.impl)))

cdef class IDIM(object):
  def __copyctor__(self, IDIM other):
    self.impl = c_rbdyn.IDIM(other.impl)
  def __mbctor__(self, MultiBody mb):
    self.impl = c_rbdyn.IDIM(deref(mb.impl))
  def __cinit__(self, *arg):
    if len(arg) == 0:
      self.impl = c_rbdyn.IDIM()
    elif len(arg) == 1:
      if isinstance(arg[0], IDIM):
        self.__copyctor__(arg[0])
      elif isinstance(arg[0], MultiBody):
        self.__mbctor__(arg[0])
    else:
      raise TypeError("Invalid arguments passed to IDIM ctor")
  def computeY(self, MultiBody mb, MultiBodyConfig mbc):
    self.impl.sComputeY(deref(mb.impl), deref(mbc.impl))
  def Y(self):
    return eigen.MatrixXdFromC(self.impl.Y())

def copy_reg_pickle():
  # Python 2/3 support
  # Try to import copyreg first (Python 3)
  # Otherwise import copy_reg (Python 2)
  try:
    import copyreg
  except ImportError:
    import copy_reg as copyreg

  # Register SVA pickle needed by RBDyn
  import sva
  sva.copy_reg_pickle()

  copyreg.pickle(Body, Body.pickle)
  copyreg.pickle(Joint, Joint.pickle)
  copyreg.pickle(MultiBody, MultiBody.pickle)
