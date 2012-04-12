import eigen3
import rbdyn as rbd
from graph.multibody import GraphicMultiBody

from mayavi import mlab

@mlab.animate(delay=10)
def run(mb, mbc):
  g = GraphicMultiBody(mb)

  fd = rbd.ForwardDynamics(mb)
  rbd.forwardKinematics(mb, mbc)
  g.draw(mb, mbc)

  i = 0
  while True:
    if i == 9:
      i = 0
      g.draw(mb, mbc)
      yield

    rbd.forwardKinematics(mb, mbc)
    rbd.forwardVelocity(mb, mbc)
    fd.forwardDynamics(mb, mbc)

    rbd.eulerIntegration(mb, mbc, 0.001)
    i = i + 1



def main():
  from robots.test_bots import make_XZX_ARM

  mb, mbc, mbg = make_XZX_ARM()

  mbc.q = [[], [1.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.]]
  mbc.jointTorque = [[], [0.], [0.], [0.]]

  a = run(mb, mbc)



if __name__ == '__main__':
  main()

