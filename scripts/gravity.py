import rbdyn as rbd
from eigen3 import Vector3d
from graph.multibody import GraphicMultiBody

from mayavi import mlab

@mlab.animate(delay=33)
def run(mb, mbc):
  g = GraphicMultiBody(mb)

  id = rbd.InverseDynamics(mb)
  fd = rbd.ForwardDynamics(mb)
  rbd.forwardKinematics(mb, mbc)
  g.draw(mb, mbc)

  mbcID = rbd.MultiBodyConfig(mbc)
  # mbcID.gravity = Vector3d(0., 0., 0.)

  while True:
    for i in range(33):
      rbd.forwardKinematics(mb, mbc)
      rbd.forwardVelocity(mb, mbc)

      mbcID.q = mbc.q
      rbd.forwardKinematics(mb, mbcID)
      rbd.forwardVelocity(mb, mbcID)

      id.inverseDynamics(mb, mbcID)

      mbc.jointTorque = mbcID.jointTorque
      fd.forwardDynamics(mb, mbc)

      rbd.eulerIntegration(mb, mbc, 0.001)

    g.draw(mb, mbc)
    # g.drawVel(mb, mbc)
    g.render()

    yield



def mainXZX():
  from robots.test_bots import make_XZX_ARM

  mb, mbc, mbg = make_XZX_ARM()

  mbc.q = [[], [1.], [0.], [0.]]

  a = run(mb, mbc)



def main3S():
  from robots.test_bots import make_3S_ARM
  from eigen3 import Quaterniond, Vector3d
  import numpy as np

  mb, mbc, mbg = make_3S_ARM()

  q1 = Quaterniond(np.pi/2., Vector3d.UnitX())
  q2 = Quaterniond(np.pi/4., Vector3d.UnitY())
  mbc.q = [[],
           [q1.w(), q1.x(), q1.y(), q1.z()],
           [q2.w(), q2.x(), q2.y(), q2.z()],
           [1., 0., 0., 0]]

  a = run(mb, mbc)



if __name__ == '__main__':
  mainXZX()

