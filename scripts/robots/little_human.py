import os

import numpy as np

from eigen3 import Vector3d, Vector6d, toEigen
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


rleg1_id = 1
rleg2_id = 2
rleg3_id = 3
rleg4_id = 4
rleg5_id = 5
rleg6_id = 6

rarm1_id = 7
rarm2_id = 8
rarm3_id = 9
rarm4_id = 10

lleg1_id = 11
lleg2_id = 12
lleg3_id = 13
lleg4_id = 14
lleg5_id = 15
lleg6_id = 16

larm1_id = 17
larm2_id = 18
larm3_id = 19
larm4_id = 20

body1_id = 21
body2_id = 22

head1_id = 23
head2_id = 24


def make_joint_link(mass, com, inertia, id, prefix, suffix, fixed=False):
  mass = mass
  com = com*1e-3
  inertia = toEigen(inertia)*1e-6

  # Transform rotational inertia to body origin
  # I_o = I_c + mc_x*c_x^T
  I_o = inertia + sva.vector3ToCrossMatrix(mass*com)*sva.vector3ToCrossMatrix(com).transpose()

  b = rbd.Body(mass, com, I_o, id, '%s_LINK%s' % (prefix, suffix))

  jType = rbd.Joint.Fixed if fixed else rbd.Joint.RevZ
  j = rbd.Joint(jType, True, id, '%s_JOINT%s' % (prefix, suffix))

  bodies.append(b)
  joints.append(j)

  return b, j



# mass property
# rleg
rleg1_m = (5.87e-02, Vector3d(-3.015, -2.2399, 3.35846e1),
           np.mat([[5.98251e+01, 4.07692e-01, 5.70458e+00],
                   [4.07692e-01, 4.32046e+01,-2.93763e+00],
                   [5.70458e+00,-2.93763e+00, 2.95761e+01]]))

rleg2_m = (2.10e-01, Vector3d(-2.72697e+00, 1.13482e+00, 2.37780e+00),
           np.mat([[6.94143e+01,-8.93597e-01, 4.50011e-01],
                   [-8.93597e-01, 5.87096e+01, 5.13879e-01],
                   [4.50011e-01, 5.13879e-01, 5.47365e+01]]))

rleg3_m = (5.19e-01, Vector3d(-6.98208e+01, 6.16508e+00,-3.26392e+00),
           np.mat([[3.33437e+02,-1.04660e+01, 2.86176e+01],
                   [-1.04660e+01, 7.15435e+02,-1.85334e+01],
                   [2.86176e+01,-1.85334e+01, 6.69659e+02]]))

rleg4_m = (3.28e-01, Vector3d(-5.08521e+01, 5.36785e+00,-4.69750e+00),
           np.mat([[2.14004e+02,-2.63054e+01,-1.52711e+01],
                   [-2.63054e+01, 3.34738e+02,-9.48167e+00],
                   [-1.52711e+01,-9.48167e+00, 2.73483e+02]]))

rleg5_m = (1.95e-01, Vector3d(2.93669e+00, 3.66853e+00,-1.33746e+00),
           np.mat([[6.35459e+01, 2.96980e-01,-5.85740e-01],
                   [2.96980e-01, 5.29750e+01,-9.54911e-01],
                   [-5.85740e-01,-9.54911e-01, 5.27906e+01]]))

rleg6_m = (1.54e-01, Vector3d(-2.92236e+01, 3.24167e+00, 8.27642e+00),
           np.mat([[2.19262e+02, 3.30179e+00, 1.63411e+01],
                   [3.30179e+00, 1.78310e+02,-3.47988e+00],
                   [1.63411e+01,-3.47988e+00, 8.20349e+01]]))


# rarm
rarm1_m = (2.10e-01, Vector3d(8.28367e-02, 6.66808e+00, 6.34824e+00),
           np.mat([[8.66688e+01, 6.58521e-02,-5.21141e-01],
                   [6.58521e-02, 6.69963e+01, 3.24962e+00],
                   [-5.21141e-01, 3.24962e+00, 7.29623e+01]]))

rarm2_m = (2.34e-01, Vector3d(-6.57560e-01,-4.63848e+01,-3.77395e+00),
           np.mat([[1.90208e+02, 4.38002e+00,-1.91098e-01],
                   [4.38002e+00, 9.55647e+01, 1.09480e+01],
                   [-1.91098e-01, 1.09480e+01, 1.66913e+02]]))

rarm3_m = (1.965e-01, Vector3d(-2.72189e+00,-1.60245e+00,-2.20452e+00),
           np.mat([[6.49972e+01,-1.21159e-01, 1.21288e+00],
                   [-1.21159e-01, 5.29142e+01, 6.02001e-01],
                   [1.21288e+00, 6.02001e-01, 6.80659e+01]]))

rarm4_m = (2.16e-01, Vector3d(1.00237e+00, 6.64225e+01,-6.15512e-01),
           np.mat([[3.60567e+02,-4.53239e-01, 8.95113e-01],
                   [-4.53239e-01, 1.03381e+02, 2.20497e+01],
                   [8.95113e-01, 2.20497e+01, 3.36596e+02]]))


# lleg
lleg1_m = (5.87e-02, Vector3d(3.00449e+00,-2.25088e+00, 3.35846e+01),
           np.mat([[5.98222e+01,-3.85905e-01,-5.71594e+00],
                   [-3.85905e-01, 4.32083e+01,-2.91155e+00],
                   [-5.71594e+00,-2.91155e+00, 2.95769e+01]]))

lleg2_m = (2.10e-01, Vector3d(-2.71179e+00,-1.24723e+00, 2.38665e+00),
           np.mat([[6.93238e+01, 5.11967e-01, 4.43105e-01],
                   [5.11967e-01, 5.85697e+01,-6.18331e-01],
                   [4.43105e-01,-6.18331e-01, 5.45564e+01]]))

lleg3_m = (5.19e-01, Vector3d(-6.99224e+01, 6.27642e+00, 3.31502e+00),
           np.mat([[3.33124e+02,-1.04299e+01,-2.93471e+01],
                   [-1.04299e+01, 7.14361e+02, 2.11288e+01],
                   [-2.93471e+01, 2.11288e+01, 6.69044e+02]]))

lleg4_m = (3.28e-01, Vector3d(-5.08386e+01, 5.37031e+00, 4.87773e+00),
           np.mat([[2.10989e+02,-2.62826e+01, 1.58357e+01],
                   [-2.62826e+01, 3.31772e+02, 8.60718e+00],
                   [1.58357e+01, 8.60718e+00, 2.73517e+02]]))

lleg5_m = (1.95e-01, Vector3d(2.92057e+00, 3.69219e+00, 1.19731e+00),
           np.mat([[6.35542e+01, 2.67585e-01, 8.67380e-01],
                   [2.67585e-01, 5.31460e+01, 8.92463e-01],
                   [8.67380e-01, 8.92463e-01, 5.26905e+01]]))

lleg6_m = (1.54e-01, Vector3d(-2.92236e+01,-3.34620e+00, 8.27642e+00),
           np.mat([[2.19156e+02,-2.72205e+00, 1.63411e+01],
                   [-2.72205e+00, 1.78310e+02, 2.84694e+00],
                   [1.63411e+01, 2.84694e+00, 8.19287e+01]]))


# larm
larm1_m = (2.10e-01, Vector3d(-3.64197e-02, 6.64174e+00,-6.34825e+00),
           np.mat([[8.66760e+01, 8.54331e-02, 2.13651e-01],
                   [8.54331e-02, 6.69975e+01,-3.14870e+00],
                   [2.13651e-01,-3.14870e+00, 7.29706e+01]]))

larm2_m = (2.34e-01, Vector3d(7.47218e-01,-4.63619e+01,-3.77403e+00),
           np.mat([[1.90701e+02,-4.53865e+00, 4.72738e-01],
                   [-4.53865e+00, 9.55349e+01, 1.08196e+01],
                   [4.72738e-01, 1.08196e+01, 1.67377e+02]]))

larm3_m = (1.965e-01, Vector3d(-2.70991e+00, 1.60227e+00,-2.09019e+00),
           np.mat([[6.51094e+01, 1.24947e-01, 1.66316e+00],
                   [1.24947e-01, 5.30426e+01,-5.66321e-01],
                   [1.66316e+00,-5.66321e-01, 6.81049e+01]]))

larm4_m = (2.16e-01, Vector3d(9.62091e-01, 6.64766e+01, 5.58629e-01),
           np.mat([[3.59070e+02,-7.34962e-01,-7.26630e-01],
                   [-7.34962e-01, 1.03303e+02,-2.22301e+01],
                   [-7.26630e-01,-2.22301e+01, 3.35228e+02]]))


# body
body1_cwb_m = (3.26e+00, Vector3d(-4.94890e+01, 8.92912e-01, 8.41741e+01),
               np.mat([[1.21947e+04, 2.95479e+02, 6.84001e+02],
                       [2.95479e+02, 1.75644e+04, 1.45795e+02],
                       [6.84001e+02, 1.45795e+02, 1.56708e+04]]))

body1_c_m = (2.40e+00, Vector3d(-6.86488e+01, 1.25142e+00, 8.45105e+01),
             np.mat([[1.09918e+04, 2.31087e+02, 6.25410e+02],
                     [2.31087e+02, 1.35490e+04, 1.53588e+02 ],
                     [6.25410e+02, 1.53588e+02, 1.16614e+04  ]]))

body1_m = (1.80e+00, Vector3d(-4.80004e+01, 4.06385e-01, 8.76728e+01),
           np.mat([[8.99663e+03, 5.16347e+01, 9.51899e+02 ],
                   [5.16347e+01, 9.07702e+03, 8.86144e+01],
                   [9.51899e+02, 8.86144e+01, 7.61051e+03]]))

body2_m = (4.17e-01, Vector3d(-3.22707e+01, 1.11375e+01,-3.29172e-01),
            np.mat([[6.60777e+02, 6.74647e+00, 4.93444e+00],
                    [6.74647e+00, 6.42117e+02,-1.51777e+00],
                    [4.93444e+00,-1.51777e+00, 1.70178e+02]]))


# head1_m
head1_m = (7.10e-02, Vector3d(5.39731e+00,-1.58067e+00, 5.64034e+01),
           np.mat([[6.90569e+01, 1.08546e+00,-9.83691e+00],
                   [1.08546e+00, 6.41244e+01, 5.08942e+00],
                   [-9.83691e+00, 5.08942e+00, 3.33541e+01]]))

head2_m = (4.12e-01, Vector3d(-1.03952e+00,-5.17985e+00, 1.82267e-02),
           np.mat([[5.52356e+02, 7.77318e+00, 3.17868e+00],
                   [7.77318e+00, 7.32140e+02, 4.57784e-01],
                   [3.17868e+00, 4.57784e-01, 6.82132e+02]]))



def bound(mb):
  low  = np.array([-91., -31., -82., -1., -61., -25., -91., -96., -91., -115.,
                   -31., -21., -82., -1., -61., -25., -91., -1., -91., -115.,
                   1.])

  low += 1.

  upp = np.array([31., 21., 71., 130., 58.6076555,  25., 151., 1., 91., 1.,
                  91., 31., 71., 130., 61., 25., 151., 96., 91., 1.,
                  90.])

  upp -= 1.

  low = np.deg2rad(low)
  upp = np.deg2rad(upp)

  ql = [[]]*mb.nrJoints()
  qu = [[]]*mb.nrJoints()

  for i,(l, u) in enumerate(zip(low, upp)):
    id = i + 1
    ql[mb.jointIndexById(id)] = [l]
    qu[mb.jointIndexById(id)] = [u]


  return (ql, qu)



def ressources(path):
  objPath = path + os.sep + 'mesh' + os.sep
  stpbvPath = path + os.sep + 'stpbv' + os.sep
  convexPath = path + os.sep + 'convex' + os.sep

  id2files = {
    body1_id:'BODY_LINK01',
    body2_id:'BODY_LINK02',

    head1_id:'HEAD_LINK01',
    head2_id:'HEAD_LINK02',

    rarm1_id:'RARM_LINK01',
    rarm2_id:'RARM_LINK02',
    rarm3_id:'RARM_LINK03',
    rarm4_id:'RARM_LINK04',

    larm1_id:'LARM_LINK01',
    larm2_id:'LARM_LINK02',
    larm3_id:'LARM_LINK03',
    larm4_id:'LARM_LINK04',

    rleg1_id:'RLEG_LINK01',
    rleg2_id:'RLEG_LINK02',
    rleg3_id:'RLEG_LINK03',
    rleg4_id:'RLEG_LINK04',
    rleg5_id:'RLEG_LINK05',
    rleg6_id:'RLEG_LINK06',

    lleg1_id:'LLEG_LINK01',
    lleg2_id:'LLEG_LINK02',
    lleg3_id:'LLEG_LINK03',
    lleg4_id:'LLEG_LINK04',
    lleg5_id:'LLEG_LINK05',
    lleg6_id:'LLEG_LINK06'
  }

  # obj
  objFiles = {}
  for id, name in id2files.items():
    objFiles[id] = [objPath + name + '.obj']

  # stpbv
  stpbvFiles = {}
  for id, name in id2files.items():
    stpbvFiles[id] = stpbvPath + name + '.txt'

  # convex
  convexFiles = {}
  for id, name in id2files.items():
    convexFiles[id] = convexPath + name + '.txt'

  return objFiles, stpbvFiles, convexFiles



def robot(fixed=True):
  rleg1_b, rleg1_j = make_joint_link(rleg1_m[0], rleg1_m[1], rleg1_m[2], rleg1_id, 'RLEG', 1)
  rleg2_b, rleg2_j = make_joint_link(rleg2_m[0], rleg2_m[1], rleg2_m[2], rleg2_id, 'RLEG', 2)
  rleg3_b, rleg3_j = make_joint_link(rleg3_m[0], rleg3_m[1], rleg3_m[2], rleg3_id, 'RLEG', 3)
  rleg4_b, rleg4_j = make_joint_link(rleg4_m[0], rleg4_m[1], rleg4_m[2], rleg4_id, 'RLEG', 4)
  rleg5_b, rleg5_j = make_joint_link(rleg5_m[0], rleg5_m[1], rleg5_m[2], rleg5_id, 'RLEG', 5)
  rleg6_b, rleg6_j = make_joint_link(rleg6_m[0], rleg6_m[1], rleg6_m[2], rleg6_id, 'RLEG', 6)

  rarm1_b, rarm1_j = make_joint_link(rarm1_m[0], rarm1_m[1], rarm1_m[2], rarm1_id, 'RARM', 1)
  rarm2_b, rarm2_j = make_joint_link(rarm2_m[0], rarm2_m[1], rarm2_m[2], rarm2_id, 'RARM', 2)
  rarm3_b, rarm3_j = make_joint_link(rarm3_m[0], rarm3_m[1], rarm3_m[2], rarm3_id, 'RARM', 3)
  rarm4_b, rarm4_j = make_joint_link(rarm4_m[0], rarm4_m[1], rarm4_m[2], rarm4_id, 'RARM', 4)

  lleg1_b, lleg1_j = make_joint_link(lleg1_m[0], lleg1_m[1], lleg1_m[2], lleg1_id, 'LLEG', 1)
  lleg2_b, lleg2_j = make_joint_link(lleg2_m[0], lleg2_m[1], lleg2_m[2], lleg2_id, 'LLEG', 2)
  lleg3_b, lleg3_j = make_joint_link(lleg3_m[0], lleg3_m[1], lleg3_m[2], lleg3_id, 'LLEG', 3)
  lleg4_b, lleg4_j = make_joint_link(lleg4_m[0], lleg4_m[1], lleg4_m[2], lleg4_id, 'LLEG', 4)
  lleg5_b, lleg5_j = make_joint_link(lleg5_m[0], lleg5_m[1], lleg5_m[2], lleg5_id, 'LLEG', 5)
  lleg6_b, lleg6_j = make_joint_link(lleg6_m[0], lleg6_m[1], lleg6_m[2], lleg6_id, 'LLEG', 6)

  larm1_b, larm1_j = make_joint_link(larm1_m[0], larm1_m[1], larm1_m[2], larm1_id, 'LARM', 1)
  larm2_b, larm2_j = make_joint_link(larm2_m[0], larm2_m[1], larm2_m[2], larm2_id, 'LARM', 2)
  larm3_b, larm3_j = make_joint_link(larm3_m[0], larm3_m[1], larm3_m[2], larm3_id, 'LARM', 3)
  larm4_b, larm4_j = make_joint_link(larm4_m[0], larm4_m[1], larm4_m[2], larm4_id, 'LARM', 4)

  body1_b, body1_j = make_joint_link(body1_c_m[0], body1_c_m[1], body1_c_m[2], body1_id, 'BODY', 1)
  body2_b, body2_j = make_joint_link(body2_m[0], body2_m[1], body2_m[2], body2_id, 'BODY', 2)

  head1_b, head1_j = make_joint_link(head1_m[0], head1_m[1], head1_m[2], head1_id, 'HEAD', 1, True)
  head2_b, head2_j = make_joint_link(head2_m[0], head2_m[1], head2_m[2], head2_id, 'HEAD', 2, True)


  mbg = rbd.MultiBodyGraph()

  global bodies
  for b in bodies:
    mbg.addBody(b)
  bodies = []

  global joints
  for j in joints:
    mbg.addJoint(j)
  joints = []

  # upper part
  chestT = dh.dhToTransform(-BODY_LINK2, 0., BODY_LINK1, 0.)

  # head
  mbg.linkBodies(body1_b.id(), dh.dhToTransform(0., 0., HEAD_LINK1, 0.)*chestT,
                 head1_b.id(), sva.PTransform.Identity(), head1_j.id())
  mbg.linkBodies(head1_b.id(), dh.dhToTransform(HEAD_LINK2, np.deg2rad(-90.), 0., 0.),
                 head2_b.id(), sva.PTransform.Identity(), head2_j.id())


  # larm
  larmT = dh.dhToTransform(0., np.deg2rad(-90.), ARM_LINK1, 0.)*chestT
  mbg.linkBodies(body1_b.id(), dh.dhToTransform(0., 0., 0., 0.)*larmT,
                 larm1_b.id(), sva.PTransform.Identity(), larm1_j.id())
  mbg.linkBodies(larm1_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., np.deg2rad(90)),
                 larm2_b.id(), sva.PTransform.Identity(), larm2_j.id())
  mbg.linkBodies(larm2_b.id(), dh.dhToTransform(0., np.deg2rad(90.), ARM_LINK2, np.deg2rad(90)),
                 larm3_b.id(), sva.PTransform.Identity(), larm3_j.id())
  mbg.linkBodies(larm3_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., 0.),
                 larm4_b.id(), sva.PTransform.Identity(), larm4_j.id())


  # rarm
  rarmT = dh.dhToTransform(0., np.deg2rad(-90.), -ARM_LINK1, 0.)*chestT
  mbg.linkBodies(body1_b.id(), dh.dhToTransform(0., 0., 0., 0.)*rarmT,
                 rarm1_b.id(), sva.PTransform.Identity(), rarm1_j.id())
  mbg.linkBodies(rarm1_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., np.deg2rad(90)),
                 rarm2_b.id(), sva.PTransform.Identity(), rarm2_j.id())
  mbg.linkBodies(rarm2_b.id(), dh.dhToTransform(0., np.deg2rad(90.), ARM_LINK2, np.deg2rad(90)),
                 rarm3_b.id(), sva.PTransform.Identity(), rarm3_j.id())
  mbg.linkBodies(rarm3_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., 0.),
                 rarm4_b.id(), sva.PTransform.Identity(), rarm4_j.id())

  # lower part
  # body
  mbg.linkBodies(body1_b.id(), dh.dhToTransform(0., np.deg2rad(90.), 0., 0.),
                 body2_b.id(), sva.PTransform.Identity(), body1_j.id())

  # legs
  leg0T =  dh.dhToTransform(-WAIST_LINK2, np.deg2rad(-90.), -WAIST_LINK1, np.deg2rad(90))

  # lleg
  mbg.linkBodies(body2_b.id(), dh.dhToTransform(LEG_LINK1, 0., 0., 0.)*leg0T,
                 lleg1_b.id(), sva.PTransform.Identity(), lleg1_j.id())
  mbg.linkBodies(lleg1_b.id(), dh.dhToTransform(0., np.deg2rad(90), 0., np.deg2rad(90)),
                 lleg2_b.id(), sva.PTransform.Identity(), lleg2_j.id())
  mbg.linkBodies(lleg2_b.id(), dh.dhToTransform(0., np.deg2rad(90), 0., 0.),
                 lleg3_b.id(), sva.PTransform.Identity(), lleg3_j.id())
  mbg.linkBodies(lleg3_b.id(), dh.dhToTransform(-LEG_LINK2, 0., 0., 0.),
                 lleg4_b.id(), sva.PTransform.Identity(), lleg4_j.id())
  mbg.linkBodies(lleg4_b.id(), dh.dhToTransform(-LEG_LINK3, 0., 0., 0.),
                 lleg5_b.id(), sva.PTransform.Identity(), lleg5_j.id())
  mbg.linkBodies(lleg5_b.id(), dh.dhToTransform(0., np.deg2rad(-90), 0., 0.),
                 lleg6_b.id(), sva.PTransform.Identity(), lleg6_j.id())
  # rleg
  mbg.linkBodies(body2_b.id(), dh.dhToTransform(-LEG_LINK1, 0., 0., 0.)*leg0T,
                 rleg1_b.id(), sva.PTransform.Identity(), rleg1_j.id())
  mbg.linkBodies(rleg1_b.id(), dh.dhToTransform(0., np.deg2rad(90), 0., np.deg2rad(90)),
                 rleg2_b.id(), sva.PTransform.Identity(), rleg2_j.id())
  mbg.linkBodies(rleg2_b.id(), dh.dhToTransform(0., np.deg2rad(90), 0., 0.),
                 rleg3_b.id(), sva.PTransform.Identity(), rleg3_j.id())
  mbg.linkBodies(rleg3_b.id(), dh.dhToTransform(-LEG_LINK2, 0., 0., 0.),
                 rleg4_b.id(), sva.PTransform.Identity(), rleg4_j.id())
  mbg.linkBodies(rleg4_b.id(), dh.dhToTransform(-LEG_LINK3, 0., 0., 0.),
                 rleg5_b.id(), sva.PTransform.Identity(), rleg5_j.id())
  mbg.linkBodies(rleg5_b.id(), dh.dhToTransform(0., np.deg2rad(-90), 0., 0.),
                 rleg6_b.id(), sva.PTransform.Identity(), rleg6_j.id())



  mb = mbg.makeMultiBody(21, fixed)

  mbc = rbd.MultiBodyConfig(mb)

  zeroParam = []
  zeroDoF = []

  if fixed:
    zeroParam.append([])
    zeroDoF.append([])
  else:
    zeroParam.append([1., 0., 0., 0., 0., 0., 0.])
    zeroDoF.append([0., 0., 0., 0., 0., 0.])

  for i in range(1, mb.nrJoints()):
    zeroParam.append([0.]*mb.joint(i).params())
    zeroDoF.append([0.]*mb.joint(i).dof())

  mbc.q = zeroParam
  mbc.alpha = zeroDoF
  mbc.alphaD = zeroDoF
  mbc.jointTorque = zeroDoF

  mbc.force = [sva.ForceVec(Vector6d.Zero())]*mb.nrBodies()

  return mb, mbc, mbg

