import rbdyn as rbd
from graph.multibody import GraphicMultiBody

from mayavi import mlab

@mlab.animate(delay=33)
def run(mb, mbc):
  g = GraphicMultiBody(mb)

  fd = rbd.ForwardDynamics(mb)
  rbd.forwardKinematics(mb, mbc)
  g.draw(mb, mbc)

  while True:
    for i in range(33):
      rbd.forwardKinematics(mb, mbc)
      rbd.forwardVelocity(mb, mbc)
      fd.forwardDynamics(mb, mbc)

      rbd.eulerIntegration(mb, mbc, 0.001)

    g.draw(mb, mbc)
    g.drawVel(mb, mbc)
    g.render()

    yield



def main():
  from robots.test_bots import make_XZX_ARM

  mb, mbc, mbg = make_XZX_ARM()

  mbc.q = [[], [1.], [0.], [0.]]

  a = run(mb, mbc)



if __name__ == '__main__':
  main()

