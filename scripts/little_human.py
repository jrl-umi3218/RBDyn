import numpy as np

from mayavi import mlab
from tvtk.api import tvtk

from eigen3 import *
import rbdyn as rbd

from graph.multibody import GraphicMultiBody
from graph.geometry import MeshGeometry

from robots.little_human import make_little_human

from mayavi.tools.animator import Animator

from task.tasks import PositionTask, PostureTask
from task.solver import WLSSolver

class Controller(object):
  def __init__(self, mb, mbc, geom):
    self.mb = mb
    self.mbc = mbc
    self.geom = geom

    self.fd = rbd.ForwardDynamics(self.mb)

    rbd.forwardKinematics(self.mb, self.mbc)

    self.graph = GraphicMultiBody(mb, geom)
    self.graph.draw(self.mb, self.mbc)
    self.graph.render()

    rhandPos = self.mb.bodyIndexById(10)
    pos = list(self.mbc.bodyPosW)[rhandPos].translation()
    obj = Vector3d(pos[0] + 0.10, pos[1] + 0.10, pos[2])

    self.solver = WLSSolver()
    posTask = PositionTask(self.mb, 10, obj)
    postureTask = PostureTask(self.mb, VectorXd.Zero(mb.nrDof()))

    self.postTask = self.solver.addTask(posTask, 100.)
    self.postureTask = self.solver.addTask(postureTask, 1.)



  def run(self):
    while True:
      for i in range(33):
        rbd.forwardKinematics(self.mb, self.mbc)
        rbd.forwardVelocity(self.mb, self.mbc)

        self.solver.solve(self.mb, self.mbc)

        #self.fd.forwardDynamics(self.mb, self.mbc)

        rbd.eulerIntegration(self.mb, self.mbc, 0.001)

      self.graph.draw(self.mb, self.mbc)
      self.graph.render()
      yield


if __name__ == '__main__':
  mb, mbc, mbg, objfile = make_little_human('../../robots/hoap_3/mesh/')

  mbc.gravity = Vector3d(0., 0., 9.81)

  geom = MeshGeometry(mb, objfile)

  cont = Controller(mb, mbc, geom)

  a = Animator(33, cont.run().next)
  a.timer.Stop()
  a.edit_traits()

