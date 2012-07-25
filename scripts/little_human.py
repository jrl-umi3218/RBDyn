import numpy as np

from mayavi import mlab
from tvtk.api import tvtk

from eigen3 import *
import spacevecalg as sva
import rbdyn as rbd

from graph.multibody import GraphicMultiBody
from graph.geometry import DefaultGeometry

from robots.little_human import make_little_human

from mayavi.tools.animator import Animator

class Controller(object):
  def __init__(self, mb, mbc, geom):
    self.mb = mb
    self.mbc = mbc
    self.geom = geom

    self.id = rbd.InverseDynamics(mb)
    self.fd = rbd.ForwardDynamics(mb)

    self.mbcID = rbd.MultiBodyConfig(mbc)

    rbd.forwardKinematics(mb, mbc)

    self.graph = GraphicMultiBody(mb, geom)
    self.graph.draw(self.mb, self.mbc)
    self.graph.render()


  def run(self):
    nb = 0
    while True:
      for i in range(330):
        rbd.forwardKinematics(self.mb, self.mbc)
        rbd.forwardVelocity(self.mb, self.mbc)
        self.fd.forwardDynamics(self.mb, self.mbc)

        rbd.eulerIntegration(self.mb, self.mbc, 0.0001)

      q = rbd.paramToVector(self.mb, self.mbc.q)
      if np.isnan(q).any():
        print "error", nb
        import sys
        sys.exit(1)
      nb += 33
      print nb

      self.graph.draw(self.mb, self.mbc)
      self.graph.render()
      yield


if __name__ == '__main__':
  mb, mbc, mbg = make_little_human()
  mbc.gravity = Vector3d(0., 0., 9.81)
  geom = DefaultGeometry(mb)

  cont = Controller(mb, mbc, geom)

  a = Animator(33, cont.run().next)
  a.timer.Stop()
  a.edit_traits()

