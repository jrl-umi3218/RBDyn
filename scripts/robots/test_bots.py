from eigen3 import Vector3d, Matrix3d, Vector6d
import spacevecalg as sva
import rbdyn as rbd

def make_leg():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')

  j0 = rbd.Joint(rbd.Joint.Spherical, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.Spherical, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.Spherical, True, 2, 'j2')

  mbg = rbd.MultiBodyGraph()

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)

  mbg.linkBodies(0, sva.PTransform(Vector3d(0., -0.15, -0.1)),
                 1, sva.PTransform.Identity(), 0)
  mbg.linkBodies(1, sva.PTransform(Vector3d(0.025, -0.35, 0.)),
                 2, sva.PTransform.Identity(), 1)
  mbg.linkBodies(2, sva.PTransform(Vector3d(0.025, -0.375, 0.)),
                 3, sva.PTransform.Identity(), 2)

  mb = mbg.makeMultiBody(0, False);

  mbc = rbd.MultiBodyConfig(mb)
  mbc.q = [[1., 0., 0., 0., 0., 0., 0.],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.],
           [1., 0., 0., 0.]]

  mbc.alpha = [[0., 0., 0., 0., 0., 0.],
               [0., 0., 0.],
               [0., 0., 0.],
               [0., 0., 0.]]

  mbc.alphaD = [[0., 0., 0., 0., 0., 0.],
                [0., 0., 0.],
                [0., 0., 0.],
                [0., 0., 0.]]

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4

  return (mb, mbc, mbg)



def make_XYZ_ARM():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')

  j0 = rbd.Joint(rbd.Joint.RevX, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.RevY, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.RevZ, True, 2, 'j2')

  mbg = rbd.MultiBodyGraph()

  to = sva.PTransform(Vector3d(0., 0.5, 0.))
  fro = sva.PTransform(Vector3d(0., -0.5, 0.))

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)

  mbg.linkBodies(0, to,
                 1, fro, 0)
  mbg.linkBodies(1, to, 2, fro, 1)
  mbg.linkBodies(2, to, 3, fro, 2)

  mb = mbg.makeMultiBody(0, True);

  mbc = rbd.MultiBodyConfig(mb)
  mbc.q = [[], [0.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4

  return (mb, mbc, mbg)



def make_XZX_ARM():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')

  j0 = rbd.Joint(rbd.Joint.RevX, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.RevZ, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.RevX, True, 2, 'j2')

  mbg = rbd.MultiBodyGraph()

  to = sva.PTransform(Vector3d(0., 0.5, 0.))
  fro = sva.PTransform(Vector3d(0., -0.5, 0.))

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)

  mbg.linkBodies(0, sva.PTransform.Identity(),
                 1, fro, 0)
  mbg.linkBodies(1, to, 2, fro, 1)
  mbg.linkBodies(2, to, 3, fro, 2)

  mb = mbg.makeMultiBody(0, True);

  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[], [0.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4


  return (mb, mbc, mbg)

