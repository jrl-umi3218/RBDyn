import math
import numpy as np

from eigen3 import toNumpy, Vector3d, Vector6d

import spacevecalg as sva
import rbdyn as rbd

from mayavi import mlab

from graph.multibody import GraphicMultiBody


def paramToPython(param):
  res = list(param)
  for i in range(len(res)):
    res[i] = list(res[i])
  return res



@mlab.animate(delay=33)
def run(mb, mbc):
  g = GraphicMultiBody(mb)

  fd = rbd.ForwardDynamics(mb)
  rbd.forwardKinematics(mb, mbc)
  g.draw(mb, mbc)


  time = 0
  while True:
    for i in range(33):
      rbd.forwardKinematics(mb, mbc)
      rbd.forwardVelocity(mb, mbc)
      fd.forwardDynamics(mb, mbc)

      rbd.eulerIntegration(mb, mbc, 0.001)


    time += 33
    g.draw(mb, mbc)
    g.drawVel(mb, mbc)
    g.render()

    yield



def mainXZX():
  from robots.test_bots import make_XZX_ARM

  mb, mbc, mbg = make_XZX_ARM()

  mbc.q = [[], [np.pi/2.], [np.pi/2.], [0.]]

  a = run(mb, mbc)



def main3S():
  from robots.test_bots import make_3S_ARM
  from eigen3 import Quaterniond, Vector3d

  mb, mbc, mbg = make_3S_ARM()

  q1 = Quaterniond(np.pi/2., Vector3d.UnitZ())
  q2 = Quaterniond(np.pi/2., Vector3d.UnitX())
  q3 = Quaterniond(0., Vector3d.UnitZ())
  # q1 = q1*Quaterniond(np.pi/2., Vector3d.UnitX())
  mbc.q = [[],
           [q1.w(), q1.x(), q1.y(), q1.z()],
           [q2.w(), q2.x(), q2.y(), q2.z()],
           [1., 0., 0., 0]]

  a = run(mb, mbc)



def main1S():
  from robots.test_bots import make_1S_ARM
  from eigen3 import Quaterniond, Vector3d

  mb, mbc, mbg = make_1S_ARM()

  q1 = Quaterniond(np.pi/2., Vector3d.UnitX())*Quaterniond(np.pi/4., Vector3d.UnitZ())
  mbc.q = [[],
           [q1.w(), q1.x(), q1.y(), q1.z()],
           []]

  a = run(mb, mbc)


def mainXZXZXZXZX():
  from robots.test_bots import make_XZXZXZXZX_ARM

  mb, mbc, mbg = make_XZXZXZXZX_ARM()

  mbc.q = [[], [np.pi/2.], [np.pi/2.], [np.pi/2.],
               [np.pi/2.], [np.pi/2.], [np.pi/2.],
               [np.pi/2.], [np.pi/2.], [np.pi/2.]]

  a = run(mb, mbc)


def mainLeg():
  from robots.test_bots import make_leg

  mb, mbc, mbg = make_leg()

  mbc.q = [[1., 0., 0., 0., 0., 0., 0.],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.]]
  mbc.jointTorque = [[0., 0., 0., 0., 0., 0.],
                     [0., 0., 0.],
                     [1., 0., 0.],
                     [0., 0., 0.]]

  mbc.gravity = Vector3d(0., 0., 0.)

  a = run(mb, mbc)


def mainLegFixed():
  from robots.test_bots import make_leg

  mb, mbc, mbg = make_leg()

  mb = mbg.makeMultiBody(0, True)
  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.]]
  mbc.alpha = [[],
               [0., 0., 0.],
               [0., 0., 0.],
               [0., 0., 0.]]
  mbc.alphaD = [[],
                [0., 0., 0.],
                [0., 0., 0.],
                [0., 0., 0.]]
  mbc.jointTorque = [[],
                     [0., 0., 0.],
                     [1., 0., 0.],
                     [0., 0., 0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4

  mbc.gravity = Vector3d(0., 0., 0.)

  a = run(mb, mbc)

if __name__ == '__main__':
  main3S()

