import numpy as np

from eigen3 import Vector3d, Matrix3d, Vector6d
import spacevecalg as sva
import rbdyn as rbd

import dh

ARM_LINK1 = 0.111
ARM_LINK2 = 0.111
ARM_LINK3 = 0.171
LEG_LINK1 = 0.039
LEG_LINK2 = 0.105
LEG_LINK3 = 0.105
LEG_LINK4 = 0.040
BODY_LINK1 = 0.125
BODY_LINK2 = 0.035
HEAD_LINK1 = 0.103
HEAD_LINK2 = 0.015
WAIST_LINK1 = 0.055
WAIST_LINK2 = 0.035

bodies = []
joints = []

def make_joint_link(mass, com, inertia, id, prefix, suffix):
  mass = mass
  com = com
  inertia = inertia
  b = rbd.Body(mass, com, inertia, id, '%s_LINK%s' % (prefix, suffix))
  j = rbd.Joint(rbd.Joint.RevZ, True, id, '%r_JOINT%r' % (prefix, suffix))

  bodies.append(b)
  joints.append(j)

  return b, j



def make_little_human():
  rleg1_b, rleg1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 1, 'RLEG', 1)
  rleg2_b, rleg2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 2, 'RLEG', 2)
  rleg3_b, rleg3_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 3, 'RLEG', 3)
  rleg4_b, rleg4_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 4, 'RLEG', 4)
  rleg5_b, rleg5_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 5, 'RLEG', 5)
  rleg6_b, rleg6_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 6, 'RLEG', 6)

  rarm1_b, rarm1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 7, 'RARM', 1)
  rarm2_b, rarm2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 8, 'RARM', 2)
  rarm3_b, rarm3_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 9, 'RARM', 3)
  rarm4_b, rarm4_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 10, 'RARM', 4)


  lleg1_b, lleg1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 11, 'LLEG', 1)
  lleg2_b, lleg2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 12, 'LLEG', 2)
  lleg3_b, lleg3_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 13, 'LLEG', 3)
  lleg4_b, lleg4_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 14, 'LLEG', 4)
  lleg5_b, lleg5_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 15, 'LLEG', 5)
  lleg6_b, lleg6_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 16, 'LLEG', 6)

  larm1_b, larm1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 17, 'LARM', 1)
  larm2_b, larm2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 18, 'LARM', 2)
  larm3_b, larm3_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 19, 'LARM', 3)
  larm4_b, larm4_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 20, 'LARM', 4)


  body1_b, body1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 21, 'BODY', 1)
  body2_b, body2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 22, 'BODY', 2)


  head1_b, head1_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 23, 'HEAD', 1)
  head2_b, head2_j = make_joint_link(1., Vector3d.Zero(), Matrix3d.Identity(), 24, 'HEAD', 2)


  mbg = rbd.MultiBodyGraph()

  for b in bodies:
    mbg.addBody(b)

  for j in joints:
    mbg.addJoint(j)

  chestT = dh.dhToTransform(-BODY_LINK2, 0., BODY_LINK1, 0.)
  head1T = dh.dhToTransform(0., 0., HEAD_LINK1, 0.)
  head2T = dh.dhToTransform(HEAD_LINK2, np.deg2rad(-90.), 0., 0.)

  mbg.linkBodies(body1_b.id(), chestT*head1T,
                 head1_b.id(), sva.PTransform.Identity(), head1_j.id())
  mbg.linkBodies(head1_b.id(), head2T,
                 head2_b.id(), sva.PTransform.Identity(), head2_j.id())


  mbg.linkBodies(body1_b.id(), chestT*dh.dhToTransform(0., np.deg2rad(-90.), ARM_LINK1, 0.),
                 larm1_b.id(), sva.PTransform.Identity(), larm1_j.id())
  mbg.linkBodies(larm1_b.id(), dh.dhToTransform(0., 0., 0., 0.),
                 larm2_b.id(), sva.PTransform.Identity(), larm2_j.id())
  mbg.linkBodies(larm2_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., np.deg2rad(90)),
                 larm3_b.id(), sva.PTransform.Identity(), larm3_j.id())
  mbg.linkBodies(larm3_b.id(), dh.dhToTransform(0., np.deg2rad(90.), ARM_LINK2, np.deg2rad(90)),
                 larm4_b.id(), sva.PTransform.Identity(), larm4_j.id())


  mbg.linkBodies(body1_b.id(), chestT*dh.dhToTransform(0., np.deg2rad(-90.), -ARM_LINK1, 0.),
                 rarm1_b.id(), sva.PTransform.Identity(), rarm1_j.id())
  mbg.linkBodies(rarm1_b.id(), dh.dhToTransform(0., 0., 0., 0.),
                 rarm2_b.id(), sva.PTransform.Identity(), rarm2_j.id())
  mbg.linkBodies(rarm2_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., np.deg2rad(90)),
                 rarm3_b.id(), sva.PTransform.Identity(), rarm3_j.id())
  mbg.linkBodies(rarm3_b.id(), dh.dhToTransform(0., np.deg2rad(90.), ARM_LINK2, np.deg2rad(90)),
                 rarm4_b.id(), sva.PTransform.Identity(), rarm4_j.id())

  mbg.linkBodies(body1_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., 0.),
                 body2_b.id(), sva.PTransform.Identity(), body1_j.id())


  mb = mbg.makeMultiBody(21, True)

  mbc = rbd.MultiBodyConfig(mb)

  zeroParam = []
  zeroDoF = []

  for i in range(mb.nrJoints()):
    zeroParam.append([0.]*mb.joint(i).params())
    zeroDoF.append([0.]*mb.joint(i).dof())

  mbc.q = zeroParam
  mbc.alpha = zeroDoF
  mbc.alphaD = zeroDoF
  mbc.jointTorque = zeroDoF

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*mb.nrBodies()

  return mb, mbc, mbg

