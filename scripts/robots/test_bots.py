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

  mbc.jointTorque = [[0., 0., 0., 0., 0., 0.],
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
  mbc.jointTorque = [[], [0.], [0.], [0.]]
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
  mbc.jointTorque = [[], [0.], [0.], [0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4


  return (mb, mbc, mbg)



def make_3S_ARM():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')

  j0 = rbd.Joint(rbd.Joint.Spherical, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.Spherical, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.Spherical, True, 2, 'j2')

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
  mbc.q = [[], [1., 0., 0., 0.], [1., 0., 0., 0.], [1., 0., 0., 0.]]
  mbc.alpha = [[], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
  mbc.alphaD = [[], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
  mbc.jointTorque = [[], [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4

  return (mb, mbc, mbg)



def make_1S_ARM():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')

  j0 = rbd.Joint(rbd.Joint.Spherical, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.Fixed, True, 1, 'j1')

  mbg = rbd.MultiBodyGraph()

  to = sva.PTransform(Vector3d(0., 0.5, 0.))
  fro = sva.PTransform(Vector3d(0., -0.5, 0.))

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)

  mbg.addJoint(j0)
  mbg.addJoint(j1)

  mbg.linkBodies(0, sva.PTransform.Identity(),
                 1, fro, 0)
  mbg.linkBodies(1, to,
                 2, fro, 1)

  mb = mbg.makeMultiBody(0, True);

  mbc = rbd.MultiBodyConfig(mb)
  mbc.q = [[], [1., 0., 0., 0.], []]
  mbc.alpha = [[], [0., 0., 0.], []]
  mbc.alphaD = [[], [0., 0., 0.], []]
  mbc.jointTorque = [[], [0., 0., 0.], []]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*3

  return (mb, mbc, mbg)

def make_XZXZXZXZX_ARM():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')
  b4 = rbd.Body(rbi, 4, 'b4')
  b5 = rbd.Body(rbi, 5, 'b5')
  b6 = rbd.Body(rbi, 6, 'b6')
  b7 = rbd.Body(rbi, 7, 'b7')
  b8 = rbd.Body(rbi, 8, 'b8')
  b9 = rbd.Body(rbi, 9, 'b9')

  j0 = rbd.Joint(rbd.Joint.RevX, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.RevZ, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.RevX, True, 2, 'j2')
  j3 = rbd.Joint(rbd.Joint.RevZ, True, 3, 'j3')
  j4 = rbd.Joint(rbd.Joint.RevX, True, 4, 'j4')
  j5 = rbd.Joint(rbd.Joint.RevZ, True, 5, 'j5')
  j6 = rbd.Joint(rbd.Joint.RevX, True, 6, 'j6')
  j7 = rbd.Joint(rbd.Joint.RevZ, True, 7, 'j7')
  j8 = rbd.Joint(rbd.Joint.RevX, True, 8, 'j8')

  mbg = rbd.MultiBodyGraph()

  to = sva.PTransform(Vector3d(0., 0.5, 0.))
  fro = sva.PTransform(Vector3d(0., -0.5, 0.))

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)
  mbg.addBody(b5)
  mbg.addBody(b6)
  mbg.addBody(b7)
  mbg.addBody(b8)
  mbg.addBody(b9)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)
  mbg.addJoint(j4)
  mbg.addJoint(j5)
  mbg.addJoint(j6)
  mbg.addJoint(j7)
  mbg.addJoint(j8)

  mbg.linkBodies(0, sva.PTransform.Identity(),
                 1, fro, 0)
  mbg.linkBodies(1, to, 2, fro, 1)
  mbg.linkBodies(2, to, 3, fro, 2)
  mbg.linkBodies(3, to, 4, fro, 3)
  mbg.linkBodies(4, to, 5, fro, 4)
  mbg.linkBodies(5, to, 6, fro, 5)
  mbg.linkBodies(6, to, 7, fro, 6)
  mbg.linkBodies(7, to, 8, fro, 7)
  mbg.linkBodies(8, to, 9, fro, 8)

  mb = mbg.makeMultiBody(0, True);

  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[], [0.], [0.], [0.],
               [0.], [0.], [0.],
               [0.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.],
                   [0.], [0.], [0.],
                   [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.],
                    [0.], [0.], [0.],
                    [0.], [0.], [0.]]
  mbc.jointTorque = [[], [0.], [0.], [0.],
                         [0.], [0.], [0.],
                         [0.], [0.], [0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*10


  return (mb, mbc, mbg)



def make_Simple_Tree():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')
  b4 = rbd.Body(rbi, 4, 'b4')

  j0 = rbd.Joint(rbd.Joint.RevZ, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.RevZ, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.RevZ, True, 2, 'j2')
  j3 = rbd.Joint(rbd.Joint.RevZ, True, 3, 'j3')

  mbg = rbd.MultiBodyGraph()

  I = sva.PTransform.Identity()

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)

  mbg.linkBodies(0, sva.PTransform(Vector3d(-1., 1., 0.)),
                 1, I, 0)
  mbg.linkBodies(1, sva.PTransform(Vector3d(-1, 0, 0.)),
                 2, I, 1)
  mbg.linkBodies(0, sva.PTransform(Vector3d(1., 1., 0.)),
                 3, I, 2)
  mbg.linkBodies(3, sva.PTransform(Vector3d(1., 0., 0.)),
                 4, I, 3)

  mb = mbg.makeMultiBody(0, True);

  mbc = rbd.MultiBodyConfig(mb)
  mbc.q = [[], [0.], [0.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.], [0.]]
  mbc.jointTorque = [[], [0.], [0.], [0.], [0.]]
  mbc.force = [sva.ForceVec(Vector6d.Zero())]*5

  return (mb, mbc, mbg)
