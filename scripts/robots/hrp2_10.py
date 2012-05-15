from eigen3 import Vector3d, Matrix3d, Vector6d
import spacevecalg as sva
import rbdyn as rbd

def make_hrp2_10_larm():
  rbi = sva.RBInertia(1., Vector3d.Zero(), Matrix3d.Identity())

  chest = rbd.Body(rbi, 10, 'CHEST')
  b0 = rbd.Body(rbi, 0, 'LARM_LINK0')
  b1 = rbd.Body(rbi, 1, 'LARM_LINK1')
  b2 = rbd.Body(rbi, 2, 'LARM_LINK2')
  b3 = rbd.Body(rbi, 3, 'LARM_LINK3')
  b4 = rbd.Body(rbi, 4, 'LARM_LINK4')
  b5 = rbd.Body(rbi, 5, 'LARM_LINK5')
  b6 = rbd.Body(rbi, 6, 'LARM_LINK6')

  j0 = rbd.Joint(rbd.Joint.RevY, True, 0, 'LARM_JOINT0')
  j1 = rbd.Joint(rbd.Joint.RevX, True, 1, 'LARM_JOINT1')
  j2 = rbd.Joint(rbd.Joint.RevZ, True, 2, 'LARM_JOINT2')
  j3 = rbd.Joint(rbd.Joint.RevY, True, 3, 'LARM_JOINT3')
  j4 = rbd.Joint(rbd.Joint.RevZ, True, 4, 'LARM_JOINT4')
  j5 = rbd.Joint(rbd.Joint.RevX, True, 5, 'LARM_JOINT5')
  j6 = rbd.Joint(rbd.Joint.RevY, True, 6, 'LARM_JOINT6')


  mbg = rbd.MultiBodyGraph()

  mbg.addBody(chest)
  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)
  mbg.addBody(b5)
  mbg.addBody(b6)

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)
  mbg.addJoint(j4)
  mbg.addJoint(j5)
  mbg.addJoint(j6)

  I = sva.PTransform.Identity()

  mbg.linkBodies(10, sva.PTransform(Vector3d(0.008, 0.250, 0.181)),
                 0, I, 0);
  mbg.linkBodies(0, sva.PTransform(Vector3d(0., 0., 0.)),
                 1, I, 1);
  mbg.linkBodies(1, sva.PTransform(Vector3d(0., 0., 0.)),
                 2, I, 2);
  mbg.linkBodies(2, sva.PTransform(Vector3d(0., 0., -0.250)),
                 3, I, 3);
  mbg.linkBodies(3, sva.PTransform(Vector3d(0., 0., -0.250)),
                 4, I, 4);
  mbg.linkBodies(4, sva.PTransform(Vector3d(0., 0., 0.)),
                 5, I, 5);
  mbg.linkBodies(5, sva.PTransform(Vector3d(0., 0., 0.)),
                 6, I, 6);

  mb = mbg.makeMultiBody(10, True);

  mbc = rbd.MultiBodyConfig(mb)

  mbc.q = [[], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
  mbc.alpha = [[], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
  mbc.alphaD = [[], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
  mbc.jointTorque = [[], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*7

  return mb, mbc, mbg

