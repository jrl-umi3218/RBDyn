from eigen3 import Vector3d, Matrix3d, Vector6d
import spacevecalg as sva
import rbdyn as rbd

def make_XYZ():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')
  b2 = rbd.Body(rbi, 2, 'b2')
  b3 = rbd.Body(rbi, 3, 'b3')
  b4 = rbd.Body(rbi, 4, 'b4')

  j0 = rbd.Joint(rbd.Joint.RevX, True, 0, 'j0')
  j1 = rbd.Joint(rbd.Joint.RevY, True, 1, 'j1')
  j2 = rbd.Joint(rbd.Joint.RevZ, True, 2, 'j2')
  j3 = rbd.Joint(rbd.Joint.Fixed, True, 3, 'j3')

  mbg = rbd.MultiBodyGraph()

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)

  I = sva.PTransform.Identity()

  mbg.linkBodies(0, I,
                 1, I, 0);
  mbg.linkBodies(1, I,
                 2, I, 1);
  mbg.linkBodies(2, I,
                 3, I, 2);
  mbg.linkBodies(3, sva.PTransform(Vector3d(1., 1., 1.)),
                 4, I, 3);

  mb = mbg.makeMultiBody(0, True)

  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[], [0.], [0.], [0.], []]
  mbc.alpha = [[], [0.], [0.], [0.], []]
  mbc.alphaD = [[], [0.], [0.], [0.], []]
  mbc.jointTorque = [[], [0.], [0.], [0.], []]

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*4

  return mb, mbc, mbg



def make_S():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  b0 = rbd.Body(rbi, 0, 'b0')
  b1 = rbd.Body(rbi, 1, 'b1')

  j0 = rbd.Joint(rbd.Joint.Spherical, True, 0, 'j0')

  mbg = rbd.MultiBodyGraph()

  mbg.addBody(b0)
  mbg.addBody(b1)

  mbg.addJoint(j0)

  I = sva.PTransform.Identity()

  mbg.linkBodies(0, I,
                 1, I, 0);

  mb = mbg.makeMultiBody(0, True)

  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[], [1., 0., 0., 0.]]
  mbc.alpha = [[], [0., 0., 0.]]
  mbc.alphaD = [[], [0., 0., 0.]]
  mbc.jointTorque = [[], [0., 0., 0.]]

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*2

  return mb, mbc, mbg


