import numpy as np

from eigen3 import *

import rbdyn as rbd


class PostureTask(object):
  def __init__(self, mb, q):
    self.q = toNumpy(q)
    self.err = None

    self.jacMat = np.mat(np.eye(mb.nrDof()))


  def update(self, mb, mbc):
    qcur = toNumpy(rbd.paramToVector(mb, mbc.q))
    self.err = self.q - qcur


  def error(self):
    return self.err


  def jacobian(self):
    return self.jacMat



class PositionTask(object):
  def __init__(self, mb, bodyId, obj):
    self.obj = toNumpy(obj)
    self.err = None
    self.bodyInd = mb.bodyIndexById(bodyId)

    self.jac = rbd.Jacobian(mb, bodyId)
    self.fullJacMat = MatrixXd(6, mb.nrDof())


  def update(self, mb, mbc):
    objCur = toNumpy(list(mbc.bodyPosW)[self.bodyInd].translation())

    self.err = self.obj - objCur
    jacMat = self.jac.jacobian(mb, mbc)
    self.jac.fullJacobian(mb, jacMat, self.fullJacMat)


  def error(self):
    return self.err


  def jacobian(self):
    return toNumpy(self.fullJacMat)[3:,:]


class NoRotationTask(object):
  def __init__(self, mb, bodyId):
    self.bodyInd = mb.bodyIndexById(bodyId)

    self.err = np.mat(np.zeros((3,1)))
    self.jac = rbd.Jacobian(mb, bodyId)
    self.fullJacMat = MatrixXd(6, mb.nrDof())


  def update(self, mb, mbc):
    jacMat = self.jac.jacobian(mb, mbc)
    self.jac.fullJacobian(mb, jacMat, self.fullJacMat)


  def error(self):
    return self.err


  def jacobian(self):
    return toNumpy(self.fullJacMat)[:3,:]



class CoMTask(object):
  def __init__(self, mb, obj):
    self.obj = toNumpy(obj)
    self.err = None

    self.jac = rbd.CoMJacobianDummy(mb)
    self.fullJacMat = MatrixXd(6, mb.nrDof())

  def update(self, mb, mbc):
    comCur = toNumpy(rbd.computeCoM(mb, mbc))

    self.err = self.obj - comCur
    self.fullJacMat = self.jac.jacobian(mb, mbc)


  def error(self):
    return self.err


  def jacobian(self):
    return toNumpy(self.fullJacMat)[3:,:]

