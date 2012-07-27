import numpy as np

from eigen3 import toEigen
import rbdyn as rbd

class WLSSolver(object):
  def __init__(self):
    self.tasks = []


  def addTask(self, task, weight):
    t = [task, weight]
    self.tasks.append(t)

    return t


  def rmTask(self, taskDef):
    self.tasks.remove(taskDef)


  def solve(self, mb, mbc):
    err = np.mat(np.empty((0, 1)))
    jac = np.mat(np.empty((0, mb.nrDof())))

    for t in self.tasks:
      t[0].update(mb, mbc)
      err = np.vstack((err, t[1]*t[0].error()))
      jac = np.vstack((jac, t[1]*t[0].jacobian()))

    #alpha1 = np.linalg.lstsq(jac, err)[0]
    alpha2 = np.linalg.pinv(jac)*err

    mbc.alpha = rbd.vectorToDof(mb, toEigen(alpha2))

