# Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# RBDyn is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RBDyn is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

from pybindgen import *
import sys



def import_sva_types(mod):
  mod.add_class('MotionVecd', foreign_cpp_namespace='sva', import_from_module='spacevecalg')
  mod.add_class('ForceVecd', foreign_cpp_namespace='sva', import_from_module='spacevecalg')
  mod.add_class('RBInertiad', foreign_cpp_namespace='sva', import_from_module='spacevecalg')
  mod.add_class('ABInertiad', foreign_cpp_namespace='sva', import_from_module='spacevecalg')
  mod.add_class('PTransformd', foreign_cpp_namespace='sva', import_from_module='spacevecalg')



def import_eigen3_types(mod):
  mod.add_class('Vector3d', foreign_cpp_namespace='Eigen', import_from_module='eigen3')
  mod.add_class('Vector6d', foreign_cpp_namespace='Eigen', import_from_module='eigen3')

  mod.add_class('Matrix3d', foreign_cpp_namespace='Eigen', import_from_module='eigen3')
  mod.add_class('Matrix6d', foreign_cpp_namespace='Eigen', import_from_module='eigen3')

  mod.add_class('MatrixXd', foreign_cpp_namespace='Eigen', import_from_module='eigen3')
  mod.add_class('VectorXd', foreign_cpp_namespace='Eigen', import_from_module='eigen3')

  mod.add_class('Quaterniond', foreign_cpp_namespace='Eigen', import_from_module='eigen3')



def build_body(bd):
  bd.add_copy_constructor()
  bd.add_constructor([param('sva::RBInertiad', 'rbInertia'),
                      param('std::string', 'name')])
  bd.add_constructor([param('double', 'mass'),
                      param('Eigen::Vector3d', 'com'),
                      param('Eigen::Matrix3d', 'inertia'),
                      param('std::string', 'name')])

  bd.add_method('name', retval('std::string'), [], is_const=True)
  bd.add_method('inertia', retval('sva::RBInertiad'), [], is_const=True)

  bd.add_binary_comparison_operator('==')
  bd.add_binary_comparison_operator('!=')

  bd.add_output_stream_operator()



def build_joint(jt):
  jt.add_enum('Type', ['Rev', 'Prism',
                       'Spherical', 'Planar', 'Cylindrical',
                       'Free', 'Fixed'])

  jt.add_copy_constructor()
  jt.add_constructor([param('rbd::Joint::Type', 'type'), param('Eigen::Vector3d', 'axis'),
                      param('bool', 'forward'), param('std::string', 'name')])
  jt.add_constructor([param('rbd::Joint::Type', 'type'), param('bool', 'forward'),
                      param('std::string', 'name')])

  jt.add_method('type', retval('rbd::Joint::Type'), [], is_const=True)

  jt.add_method('direction', retval('double'), [], is_const=True)
  jt.add_method('forward', retval('bool'), [], is_const=True)
  jt.add_method('forward', None, [param('double', 'forward')])

  jt.add_method('params', retval('int'), [], is_const=True)
  jt.add_method('dof', retval('int'), [], is_const=True)
  jt.add_method('name', retval('std::string'), [], is_const=True)

  jt.add_method('motionSubspace', retval('Eigen::MatrixXd'), [], is_const=True)

  jt.add_method('sPose', retval('sva::PTransformd'),
                [param('const std::vector<double>&', 'q')],
                throw=[dom_ex], custom_name='pose')

  jt.add_method('sMotion', retval('sva::MotionVecd'),
                [param('const std::vector<double>&', 'alpha')],
                throw=[dom_ex], custom_name='motion')

  jt.add_method('zeroParam', retval('std::vector<double>'), [], is_const=True)
  jt.add_method('zeroDof', retval('std::vector<double>'), [], is_const=True)

  jt.add_method('ZeroParam', retval('std::vector<double>'),
                [param('rbd::Joint::Type', 'type')], is_static=True)
  jt.add_method('ZeroDof', retval('std::vector<double>'),
                [param('rbd::Joint::Type', 'type')], is_static=True)


  jt.add_binary_comparison_operator('==')
  jt.add_binary_comparison_operator('!=')

  jt.add_output_stream_operator()



def build_mbg(mbg):
  mbg.add_constructor([])
  mbg.add_copy_constructor()

  mbg.add_method('addBody', None, [param('rbd::Body', 'body')], throw=[dom_ex])
  mbg.add_method('addJoint', None, [param('rbd::Joint', 'joint')], throw=[dom_ex])

  mbg.add_method('linkBodies', None,
                 [param('const std::string&', 'b1Name'),
                  param('sva::PTransformd', 'tB1'),
                  param('const std::string&', 'b2Name'),
                  param('sva::PTransformd', 'tB2'),
                  param('const std::string&', 'jointName'),
                  param('bool', 'isB1toB2', default_value='true')],
                 throw=[out_ex])

  mbg.add_method('nrNodes', retval('int'), [], is_const=True)
  mbg.add_method('nrJoints', retval('int'), [], is_const=True)

  mbg.add_method('removeJoint', None, [param('std::string&', 'rootBodyName'),
                                       param('std::string&', 'name')],
                 throw=(out_ex,))
  mbg.add_method('removeJoint', None, [param('std::string&', 'rootBodyName'),
                                       param('const std::string&', 'name')],
                 throw=(out_ex,))
  mbg.add_method('removeJoints', None, [param('std::string&', 'rootBodyName'),
                                        param('const std::vector<std::string>&', 'names')],
                 throw=(out_ex,))
  mbg.add_method('removeJoints', None, [param('std::string&', 'rootBodyName'),
                                        param('const std::vector<std::string>&', 'names')],
                 throw=(out_ex,))

  mbg.add_method('mergeSubBodies', None, [param('const std::string&', 'rootBodyName'),
                                          param('const std::string&', 'name'),
                                          param('const std::map<std::string, std::vector<double> >&', 'jointPosByName')],
                 throw=(out_ex, dom_ex))
  mbg.add_method('mergeSubBodies', None, [param('std::string&', 'rootBodyName'),
                                          param('const std::string&', 'name'),
                                          param('const std::map<std::string, std::vector<double> >&', 'jointPosByName')],
                 throw=(out_ex, dom_ex))

  mbg.add_method('makeMultiBody', retval('rbd::MultiBody'),
                 [param('std::string&', 'rootBodyName'), param('bool', 'isFixed'),
                  param('const sva::PTransformd&', 'X_0_j0', default_value='sva::PTransformd::Identity()'),
                  param('const sva::PTransformd&', 'X_b0_j0', default_value='sva::PTransformd::Identity()')],
                 throw=(out_ex,))

  mbg.add_method('makeMultiBody', retval('rbd::MultiBody'),
                 [param('std::string&', 'rootBodyName'), param('rbd::Joint::Type', 'rootJointType'),
                  param('const Eigen::Vector3d&', 'axis'),
                  param('const sva::PTransformd&', 'X_0_j0', default_value='sva::PTransformd::Identity()'),
                  param('const sva::PTransformd&', 'X_b0_j0', default_value='sva::PTransformd::Identity()')],
                 throw=(out_ex,))

  mbg.add_method('bodiesBaseTransform',
                 retval('std::map<std::string, sva::PTransformd>'),
                 [param('std::string&', 'rootBodyName'),
                  param('const sva::PTransformd&', 'X_b0_j0', default_value='sva::PTransformd::Identity()')],
                 throw=(out_ex,))

  mbg.add_method('successorJoints',
                 retval('std::map<std::string, std::vector<std::string> >'),
                 [param('std::string&', 'rootBodyName')],
                 throw=(out_ex,))

  mbg.add_method('predecessorJoint',
                 retval('std::map<std::string, std::string>'),
                 [param('const std::string&', 'rootBodyName')],
                 throw=(out_ex,))


def build_mb(mb):
  mb.add_constructor([])

  mb.add_constructor([param('std::vector<rbd::Body>', 'bodies'),
                      param('std::vector<rbd::Joint>', 'joints'),
                      param('std::vector<int>', 'pred'),
                      param('std::vector<int>', 'succ'),
                      param('std::vector<int>', 'parent'),
                      param('std::vector<sva::PTransformd>', 'Xt')])

  mb.add_copy_constructor()

  mb.add_method('nrBodies', retval('int'), [], is_const=True)
  mb.add_method('nrJoints', retval('int'), [], is_const=True)

  mb.add_method('nrParams', retval('int'), [], is_const=True)
  mb.add_method('nrDof', retval('int'), [], is_const=True)

  mb.add_method('bodies', retval('std::vector<rbd::Body>'), [], is_const=True)
  mb.add_method('sBody', retval('rbd::Body'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='body')

  mb.add_method('sBodies', None, [param('std::vector<rbd::Body>', 'bodies')],
                throw=[run_ex], custom_name='bodies')
  mb.add_method('sBody', None, [param('int', 'num'),
                                param('const rbd::Body&', 'b')],
                throw=[out_ex], custom_name='body')

  mb.add_method('joints', retval('std::vector<rbd::Joint>'), [], is_const=True)
  mb.add_method('sJoint', retval('rbd::Joint'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='joint')

  mb.add_method('predecessors', retval('std::vector<int>'), [], is_const=True)
  mb.add_method('sPredecessor', retval('int'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='predecessor')

  mb.add_method('successors', retval('std::vector<int>'), [], is_const=True)
  mb.add_method('sSuccessor', retval('int'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='successor')

  mb.add_method('parents', retval('std::vector<int>'), [], is_const=True)
  mb.add_method('sParent', retval('int'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='parent')

  mb.add_method('transforms', retval('std::vector<sva::PTransformd>'), [], is_const=True)
  mb.add_method('sTransform', retval('sva::PTransformd'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='transform')

  mb.add_method('sTransforms', None, [param('std::vector<sva::PTransformd>', 'Xt')],
                throw=[run_ex], custom_name='transforms')
  mb.add_method('sTransform', None, [param('int', 'num'),
                                     param('const sva::PTransformd&', 'X')],
                throw=[out_ex], custom_name='transform')

  mb.add_method('jointsPosInParam', retval('std::vector<int>'), [], is_const=True)
  mb.add_method('sJointPosInParam', retval('int'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='jointPosInParam')

  mb.add_method('jointsPosInDof', retval('std::vector<int>'), [], is_const=True)
  mb.add_method('sJointPosInDof', retval('int'), [param('int', 'num')],
                is_const=True, throw=[out_ex], custom_name='jointPosInDof')

  mb.add_method('sBodyIndexByName', retval('int'), [param('const std::string&', 'name')],
                is_const=True, throw=[out_ex], custom_name='bodyIndexByName')

  mb.add_method('sJointIndexByName', retval('int'), [param('const std::string&', 'name')],
                is_const=True, throw=[out_ex], custom_name='jointIndexById')



def build_mbc(mbc):
  mbc.add_constructor([])
  mbc.add_constructor([param('const rbd::MultiBody&', 'mb')])

  mbc.add_copy_constructor()

  mbc.add_method('zero', None, [param('const MultiBody&', 'mb')])

  mbc.add_instance_attribute('q', 'std::vector<std::vector<double> >')
  mbc.add_instance_attribute('alpha', 'std::vector<std::vector<double> >')
  mbc.add_instance_attribute('alphaD', 'std::vector<std::vector<double> >')

  mbc.add_instance_attribute('force', 'std::vector<sva::ForceVecd>')

  mbc.add_instance_attribute('jointConfig', 'std::vector<sva::PTransformd>')
  mbc.add_instance_attribute('jointVelocity', 'std::vector<sva::MotionVecd>')
  mbc.add_instance_attribute('jointTorque', 'std::vector<std::vector<double> >')

  mbc.add_instance_attribute('motionSubspace', 'std::vector<Eigen::MatrixXd>',
                             getter='python_motionSubspace',
                             setter='python_motionSubspace')

  mbc.add_instance_attribute('bodyPosW', 'std::vector<sva::PTransformd>')
  mbc.add_instance_attribute('parentToSon', 'std::vector<sva::PTransformd>')

  mbc.add_instance_attribute('bodyVelW', 'std::vector<sva::MotionVecd>')
  mbc.add_instance_attribute('bodyVelB', 'std::vector<sva::MotionVecd>')
  mbc.add_instance_attribute('bodyAccB', 'std::vector<sva::MotionVecd>')

  mbc.add_instance_attribute('gravity', 'Eigen::Vector3d')



def build_utility(mod):
  mod.add_function('sParamToVector', None,
                   [param('const std::vector<std::vector<double> >&', 'v'),
                    param('Eigen::VectorXd&', 'e')],
                   custom_name='paramToVector',
                   throw=[out_ex])

  mod.add_function('sVectorToParam', None,
                   [param('const Eigen::VectorXd&', 'e'),
                    param('std::vector<std::vector<double> >&', 'v')],
                   custom_name='vectorToParam',
                   throw=[out_ex])

  mod.add_function('sParamToVector', retval('Eigen::VectorXd'),
                   [param('const MultiBody&', 'mb'),
                    param('const std::vector<std::vector<double> >&', 'v')],
                   custom_name='paramToVector',
                   throw=[out_ex])

  mod.add_function('sVectorToParam', retval('std::vector<std::vector<double> >'),
                   [param('const MultiBody&', 'mb'),
                    param('const Eigen::VectorXd&', 'e')],
                   custom_name='vectorToParam',
                   throw=[out_ex])

  mod.add_function('sDofToVector', retval('Eigen::VectorXd'),
                   [param('const MultiBody&', 'mb'),
                    param('const std::vector<std::vector<double> >&', 'v')],
                   custom_name='dofToVector',
                   throw=[out_ex])

  mod.add_function('sVectorToDof', retval('std::vector<std::vector<double> >'),
                   [param('const MultiBody&', 'mb'),
                    param('const Eigen::VectorXd&', 'e')],
                   custom_name='vectorToDof',
                   throw=[out_ex])



def build_algo(mod):
  mod.add_function('sForwardKinematics', None,
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc')],
                   custom_name='forwardKinematics',
                   throw=[dom_ex])

  mod.add_function('sForwardVelocity', None,
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc')],
                   custom_name='forwardVelocity',
                   throw=[dom_ex])

  mod.add_function('sForwardAcceleration', None,
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc'),
                    param('const sva::MotionVecd&', 'A_0',
                          default_value='sva::MotionVecd(Eigen::Vector6d::Zero())')],
                   custom_name='forwardAcceleration',
                   throw=[dom_ex])

  mod.add_function('sEulerIntegration', None,
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc'),
                    param('double', 'step')],
                   custom_name='eulerIntegration',
                   throw=[dom_ex])


def build_jacobian(jac):
  jac.add_constructor([])
  jac.add_copy_constructor()
  jac.add_constructor([param('const rbd::MultiBody&', 'mb'),
                       param('const std::string&', 'bodyName'),
                       param('const Eigen::Vector3d&', 'point',
                             default_value='Eigen::Vector3d::Zero()')],
                     throw=[out_ex])

  jac.add_method('sJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const sva::PTransformd&', 'X_0_p')],
                 throw=[dom_ex], custom_name='jacobian')

  jac.add_method('sJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobian')

  jac.add_method('sJacobianDot', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobianDot')

  jac.add_method('sBodyJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='bodyJacobian')

  jac.add_method('sBodyJacobianDot', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='bodyJacobianDot')

  jac.add_method('sVectorJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'vec')],
                 throw=[dom_ex], custom_name='vectorJacobian')

  jac.add_method('sVectorBodyJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'vec')],
                 throw=[dom_ex], custom_name='vectorBodyJacobian')

  jac.add_method('sTranslateJacobian', None,
                 [param('const Eigen::MatrixXd&', 'jac'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'point'),
                  param('Eigen::MatrixXd&', 'res')],
                 throw=[dom_ex], custom_name='translateJacobian')

  jac.add_method('sFullJacobian', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const Eigen::MatrixXd&', 'jac'),
                  param('Eigen::MatrixXd&', 'res')],
                 throw=[dom_ex], custom_name='fullJacobian')

  jac.add_method('sSubMultiBody', retval('rbd::MultiBody'),
                 [param('const rbd::MultiBody&', 'mb')],
                 throw=[dom_ex], custom_name='subMultiBody',
                 is_const=True)

  jac.add_method('jointsPath', retval('std::vector<int>'),
                 [], is_const=True)

  jac.add_method('dof', retval('int'), [], is_const=True)

  jac.add_method('point', retval('Eigen::Vector3d'), [], is_const=True)
  jac.add_method('point', None, [param('const Eigen::Vector3d&', 'point')])

  jac.add_method('sVelocity', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='velocity',
                 is_const=True)

  jac.add_method('sVelocity', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const sva::PTransformd&', 'X_b_p')],
                 throw=[dom_ex], custom_name='velocity',
                 is_const=True)

  jac.add_method('sBodyVelocity', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='bodyVelocity',
                 is_const=True)

  jac.add_method('sNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='normalAcceleration',
                 is_const=True)

  jac.add_method('sNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const sva::PTransformd&', 'X_b_p'),
                  param('const sva::MotionVecd&', 'V_b_p')],
                 throw=[dom_ex], custom_name='normalAcceleration',
                 is_const=True)

  jac.add_method('sNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const std::vector<sva::MotionVecd>&', 'normalAccB'),
                  param('const sva::PTransformd&', 'X_b_p'),
                  param('const sva::MotionVecd&', 'V_b_p')],
                 throw=[dom_ex], custom_name='normalAcceleration',
                 is_const=True)

  jac.add_method('sBodyNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='bodyNormalAcceleration',
                 is_const=True)

  jac.add_method('sNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const std::vector<sva::MotionVecd>&', 'normalAccB')],
                 throw=[dom_ex], custom_name='normalAcceleration',
                 is_const=True)

  jac.add_method('sBodyNormalAcceleration', retval('sva::MotionVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const std::vector<sva::MotionVecd>&', 'normalAccB')],
                 throw=[dom_ex], custom_name='bodyNormalAcceleration',
                 is_const=True)



def build_id(id):
  id.add_constructor([])
  id.add_copy_constructor()
  id.add_constructor([param('const rbd::MultiBody&', 'mb')])


  id.add_method('sInverseDynamics', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='inverseDynamics')

  id.add_method('f', retval('std::vector<sva::ForceVecd>'), [], is_const=True)


def build_ik(ik):
  ik.add_copy_constructor()
  ik.add_constructor([param('const rbd::MultiBody&', 'mb'), param('int', 'ef_index')])

  ik.add_method('sInverseKinematics', retval('bool'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc'),
                  param('const sva::PTransformd&', 'ef_target')],
                 throw=[dom_ex], custom_name='inverseKinematics')

def build_fd(id):
  fd.add_constructor([])
  fd.add_copy_constructor()
  fd.add_constructor([param('const rbd::MultiBody&', 'mb')])


  fd.add_method('sForwardDynamics', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='forwardDynamics')

  fd.add_method('sComputeH', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='computeH')

  fd.add_method('sComputeC', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='computeC')


  fd.add_method('H', retval('const Eigen::MatrixXd&'), [], is_const=True)
  fd.add_method('C', retval('const Eigen::VectorXd&'), [], is_const=True)
  fd.add_method('inertiaSubTree', retval('std::vector<sva::RBInertiad>'), [], is_const=True)



def build_com(mod, comD, comJ):
  mod.add_function('sComputeCoM', retval('Eigen::Vector3d'),
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc')],
                   custom_name='computeCoM',
                   throw=[dom_ex])
  mod.add_function('sComputeCoMVelocity', retval('Eigen::Vector3d'),
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc')],
                   custom_name='computeCoMVelocity',
                   throw=[dom_ex])
  mod.add_function('sComputeCoMAcceleration', retval('Eigen::Vector3d'),
                   [param('const MultiBody&', 'mb'),
                    param('MultiBodyConfig&', 'mbc')],
                   custom_name='computeCoMAcceleration',
                   throw=[dom_ex])


  comD.add_constructor([])
  comD.add_copy_constructor()
  comD.add_constructor([param('const rbd::MultiBody&', 'mb')])
  comD.add_constructor([param('const rbd::MultiBody&', 'mb'),
                        param('std::vector<double>', 'weight')],
                        throw=[dom_ex])

  comD.add_method('sJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobian')

  comD.add_method('sJacobianDot', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobianDot')


  comJ.add_constructor([])
  comJ.add_copy_constructor()
  comJ.add_constructor([param('const rbd::MultiBody&', 'mb')])
  comJ.add_constructor([param('const rbd::MultiBody&', 'mb'),
                        param('std::vector<double>', 'weight')],
                        throw=[dom_ex])

  comJ.add_method('sUpdateInertialParameters', None,
                 [param('const rbd::MultiBody&', 'mb')],
                 throw=[dom_ex], custom_name='updateInertialParameters')

  comJ.add_method('weight', retval('std::vector<double>'),
                 [], is_const=True)
  comJ.add_method('sWeight', None,
                  [param('const rbd::MultiBody&', 'mb'),
                   param('std::vector<double>', 'weight')],
                  throw=[dom_ex], custom_name='weight')

  comJ.add_method('sJacobian', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobian')

  comJ.add_method('sJacobianDot', retval('Eigen::MatrixXd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('rbd::MultiBodyConfig&', 'mbc')],
                 throw=[dom_ex], custom_name='jacobianDot')

  comJ.add_method('sVelocity', retval('Eigen::Vector3d'),
                  [param('const rbd::MultiBody&', 'mb'),
                   param('const rbd::MultiBodyConfig&', 'mbc')],
                  throw=[dom_ex], custom_name='velocity',
                  is_const=True)

  comJ.add_method('sNormalAcceleration', retval('Eigen::Vector3d'),
                  [param('const rbd::MultiBody&', 'mb'),
                   param('const rbd::MultiBodyConfig&', 'mbc')],
                  throw=[dom_ex], custom_name='normalAcceleration')

  comJ.add_method('sNormalAcceleration', retval('Eigen::Vector3d'),
                  [param('const rbd::MultiBody&', 'mb'),
                   param('const rbd::MultiBodyConfig&', 'mbc'),
                   param('const std::vector<sva::MotionVecd>&', 'normalAccB')],
                  throw=[dom_ex], custom_name='normalAcceleration',
                  is_const=True)



def build_momentum(mod, mom):
  mod.add_function('sComputeCentroidalMomentum',
                   retval('sva::ForceVecd'),
                   [param('const MultiBody&', 'mb'),
                    param('const MultiBodyConfig&', 'mbc'),
                    param('const Eigen::Vector3d&', 'com')],
                   custom_name='computeCentroidalMomentum',
                   throw=[dom_ex])

  mod.add_function('sComputeCentroidalMomentumDot',
                   retval('sva::ForceVecd'),
                   [param('const MultiBody&', 'mb'),
                    param('const MultiBodyConfig&', 'mbc'),
                    param('const Eigen::Vector3d&', 'com'),
                    param('const Eigen::Vector3d&', 'comVel')],
                   custom_name='computeCentroidalMomentumDot',
                   throw=[dom_ex])

  mom.add_constructor([])
  mom.add_copy_constructor()
  mom.add_constructor([param('const rbd::MultiBody&', 'mb')])
  mom.add_constructor([param('const rbd::MultiBody&', 'mb')])
  mom.add_constructor([param('const rbd::MultiBody&', 'mb'),
                       param('std::vector<double>', 'weight')],
                      throw=[dom_ex])

  mom.add_method('sComputeMatrix', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com')],
                 custom_name='computeMatrix',
                 throw=[dom_ex])
  mom.add_method('sComputeMatrixDot', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com'),
                  param('const Eigen::Vector3d&', 'comDot')],
                 custom_name='computeMatrixDot',
                 throw=[dom_ex])
  mom.add_method('sComputeMatrixAndMatrixDot', None,
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com'),
                  param('const Eigen::Vector3d&', 'comDot')],
                 custom_name='computeMatrixAndMatrixDot',
                 throw=[dom_ex])

  mom.add_method('matrix', retval('Eigen::MatrixXd'), [], is_const=True)
  mom.add_method('matrixDot', retval('Eigen::MatrixXd'), [], is_const=True)

  mom.add_method('sMomentum', retval('sva::ForceVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com')],
                 throw=[dom_ex], custom_name='momentum',
                 is_const=True)

  mom.add_method('sNormalMomentumDot', retval('sva::ForceVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com'),
                  param('const Eigen::Vector3d&', 'comDot')],
                 throw=[dom_ex], custom_name='normalMomentumDot')

  mom.add_method('sNormalMomentumDot', retval('sva::ForceVecd'),
                 [param('const rbd::MultiBody&', 'mb'),
                  param('const rbd::MultiBodyConfig&', 'mbc'),
                  param('const Eigen::Vector3d&', 'com'),
                  param('const Eigen::Vector3d&', 'comDot'),
                  param('const std::vector<sva::MotionVecd>&', 'normalAccB')],
                 throw=[dom_ex], custom_name='normalMomentumDot',
                 is_const=True)



def build_zmp(mod):
  mod.add_function('computeCentroidalZMP',
                 retval('Eigen::Vector3d'),
                 [param('rbd::MultiBodyConfig', 'mbc'),
                  param('Eigen::Vector3d', 'com'),
                  param('Eigen::Vector3d', 'comA'),
                  param('double', 'altitude')])
  mod.add_function('computeCentroidalZMPNoGravity',
                 retval('Eigen::Vector3d'),
                 [param('Eigen::Vector3d', 'com'),
                  param('Eigen::Vector3d', 'comA'),
                  param('double', 'altitude')])
  mod.add_function('computeCentroidalZMPComplete',
                 retval('Eigen::Vector3d'),
                 [param('rbd::MultiBodyConfig', 'mbc'),
                  param('Eigen::Vector3d', 'com'),
                  param('Eigen::Vector3d', 'comA'),
                  param('double', 'altitude'),
                  param('sva::ForceVecd', 'wr_external'),
                  param('double', 'mass')])



def build_confconv(conf):
  const = conf.add_function_as_constructor('rbd::ConfigConverter::sConstructor', 'ConfigConverter*',
                                   [param('const rbd::MultiBody&', 'mb1'),
                                    param('const rbd::MultiBody&', 'mb2')])
  conf.add_copy_constructor()
  const.throw = [dom_ex]

  conf.add_method('sConvert', None,
                 [param('const rbd::MultiBodyConfig&', 'mbc1'),
                  param('rbd::MultiBodyConfig&', 'mbc2')],
                 throw=[dom_ex], custom_name='convert')

  conf.add_method('convertJoint', retval('std::vector<std::vector<double> >'),
                  [param('const std::vector<std::vector<double> >&', 'from')])
  conf.add_method('convertJoint', retval('std::vector<double>'),
                  [param('const std::vector<double>&', 'from')])



def build_idim(mod, idim):
  mod.add_function('IMPhi', retval('Eigen::MatrixXd'),
                   [param('const sva::MotionVecd&', 'mv')])
  mod.add_function('inertiaToVector', retval('Eigen::VectorXd'),
                   [param('const sva::RBInertiad&', 'rbi')])
  mod.add_function('sVectorToInertia', retval('sva::RBInertiad'),
                   [param('const Eigen::VectorXd&', 'vec')],
                   throw=[out_ex], custom_name='vectorToInertia')
  mod.add_function('multiBodyToInertialVector', retval('Eigen::VectorXd'),
                   [param('const rbd::MultiBody&', 'mb')])

  idim.add_constructor([])
  idim.add_copy_constructor()
  idim.add_constructor([param('const rbd::MultiBody&', 'mb')])


  idim.add_method('sComputeY', None,
                  [param('const rbd::MultiBody&', 'mb'),
                   param('rbd::MultiBodyConfig&', 'mbc')],
                  throw=[dom_ex], custom_name='computeY')
  idim.add_method('Y', retval('Eigen::MatrixXd'), [], is_const=True)



if __name__ == '__main__':
  if len(sys.argv) < 2:
    sys.exit(1)

  rbd = Module('_rbdyn', cpp_namespace='::rbd')
  rbd.add_include('<Body.h>')
  rbd.add_include('<Joint.h>')
  rbd.add_include('<MultiBody.h>')
  rbd.add_include('<MultiBodyConfig.h>')
  rbd.add_include('<MultiBodyGraph.h>')
  rbd.add_include('<FK.h>')
  rbd.add_include('<FV.h>')
  rbd.add_include('<FA.h>')
  rbd.add_include('<Jacobian.h>')
  rbd.add_include('<ID.h>')
  rbd.add_include('<FD.h>')
  rbd.add_include('<IK.h>')
  rbd.add_include('<EulerIntegration.h>')
  rbd.add_include('<CoM.h>')
  rbd.add_include('<Momentum.h>')
  rbd.add_include('<ZMP.h>')
  rbd.add_include('<IDIM.h>')

  dom_ex = rbd.add_exception('std::domain_error', foreign_cpp_namespace=' ',
                             message_rvalue='%(EXC)s.what()')
  run_ex = rbd.add_exception('std::runtime_error', foreign_cpp_namespace=' ',
                             message_rvalue='%(EXC)s.what()')
  out_ex = rbd.add_exception('std::out_of_range', foreign_cpp_namespace=' ',
                             message_rvalue='%(EXC)s.what()')

  # import Eigen3 and sva types
  import_eigen3_types(rbd)
  import_sva_types(rbd)

  body = rbd.add_class('Body')
  joint = rbd.add_class('Joint')
  mb = rbd.add_class('MultiBody')
  mbc = rbd.add_struct('MultiBodyConfig')
  mbg = rbd.add_class('MultiBodyGraph')
  jac = rbd.add_class('Jacobian')
  id = rbd.add_class('InverseDynamics')
  fd = rbd.add_class('ForwardDynamics')
  ik = rbd.add_class('InverseKinematics')
  comDummy = rbd.add_class('CoMJacobianDummy')
  comJac = rbd.add_class('CoMJacobian')
  momentumMat = rbd.add_class('CentroidalMomentumMatrix')
  confconv = rbd.add_class('ConfigConverter')
  idim = rbd.add_class('IDIM')

  # build list type
  rbd.add_container('std::vector<double>', 'double', 'vector')
  rbd.add_container('std::vector<int>', 'int', 'vector')
  rbd.add_container('std::vector<std::string>', 'std::string', 'vector')
  rbd.add_container('std::vector<std::vector<double> >', 'std::vector<double>', 'vector')
  rbd.add_container('std::vector<rbd::Body>', 'rbd::Body', 'vector')
  rbd.add_container('std::vector<rbd::Joint>', 'rbd::Joint', 'vector')
  rbd.add_container('std::vector<sva::PTransformd>', 'sva::PTransformd', 'vector')
  rbd.add_container('std::vector<sva::MotionVecd>', 'sva::MotionVecd', 'vector')
  rbd.add_container('std::vector<sva::ForceVecd>', 'sva::ForceVecd', 'vector')
  rbd.add_container('std::vector<sva::RBInertiad>', 'sva::RBInertiad', 'vector')
  rbd.add_container('std::vector<Eigen::MatrixXd>', 'Eigen::MatrixXd', 'vector')

  # build map type
  rbd.add_container('std::map<std::string, std::string>',
                    ('std::string', 'std::string'), 'map')
  rbd.add_container('std::map<std::string, std::vector<std::string> >',
                    ('std::string', 'std::vector<std::string>'), 'map')
  rbd.add_container('std::map<std::string, std::vector<double> >',
                    ('std::string', 'std::vector<double>'), 'map')
  rbd.add_container('std::map<std::string, sva::PTransformd>',
                    ('std::string', 'sva::PTransformd'), 'map')

  build_body(body)
  build_joint(joint)
  build_mbg(mbg)
  build_mb(mb)
  build_mbc(mbc)
  build_jacobian(jac)

  build_utility(rbd)
  build_algo(rbd)
  build_id(id)
  build_fd(fd)
  build_ik(ik)
  build_com(rbd, comDummy, comJac)
  build_momentum(rbd, momentumMat)
  build_zmp(rbd)
  build_confconv(confconv)
  build_idim(rbd, idim)

  with open(sys.argv[1], 'w') as f:
    rbd.generate(f)

