# -*- coding: utf-8 -*-
#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

import unittest

import rbdyn

urdf_model = """<robot name="XYZSarm">
  <link name="b0">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
              iyy="0.05" iyz="0.0" izz="0.001" />
    </inertial>
    <visual>
      <origin rpy="0. 0. 0." xyz=".1 .2 .3"/>
      <geometry>
        <mesh filename="test_mesh1.dae"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="test_mesh2.dae"/>
      </geometry>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
  </link>
  <link name="b1">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.5 0" />
      <mass value="5." />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
              iyy="0.05" iyz="0.0" izz="0.001" />
    </inertial>
  </link>
  <link name="b2">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.5 0" />
      <mass value="2." />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
              iyy="0.05" iyz="0.0" izz="0.001" />
    </inertial>
  </link>
  <link name="b3">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0.5 0" />
      <mass value="1.5" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
              iyy="0.05" iyz="0.0" izz="0.001" />
    </inertial>
  </link>
  <link name="b4">
    <inertial>
      <origin rpy="0 0 0" xyz="0.5 0 0" />
      <mass value="1" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
              iyy="0.05" iyz="0.0" izz="0.001" />
    </inertial>
  </link>
  <joint name="j0" type="revolute">
    <parent link="b0" />
    <child link="b1" />
    <origin rpy="0 0 0" xyz="0 1 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1" upper="1" velocity="10" effort="50" />
  </joint>
  <joint name="j1" type="revolute">
    <parent link="b1" />
    <child link="b2" />
    <origin rpy="0 0 0" xyz="0 1 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1" upper="1" velocity="10" effort="50" />
  </joint>
  <joint name="j2" type="revolute">
    <parent link="b2" />
    <child link="b3" />
    <origin rpy="0 0 0" xyz="0 1 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1" upper="1" velocity="10" effort="50" />
  </joint>
  <joint name="j3" type="continuous">
    <parent link="b1" />
    <child link="b4" />
    <origin rpy="1. 0 0" xyz="1 0 0" />
    <axis xyz="1 0 0" />
  </joint>
</robot>
"""

yaml_model = """robot:
  name: XYZSarm
  anglesInDegrees: false
  links:
    - name: b0
      inertial:
        mass: 1
        frame:
          xyz: [0, 0, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
      visual:
        - frame:
            xyz: [0.1, 0.2, 0.3]
            rpy: [0, 0, 0]
          geometry:
            mesh:
              filename: test_mesh1.dae
        - frame:
            xyz: [0, 0, 0]
            rpy: [0, 0, 0]
          geometry:
            mesh:
              filename: test_mesh2.dae
        - frame:
            xyz: [0, 0, 0]
            rpy: [0, 0, 0]
    - name: b1
      inertial:
        mass: 5
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
    - name: b2
      inertial:
        mass: 2
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
    - name: b3
      inertial:
        mass: 1.5
        frame:
          xyz: [0, 0.5, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
    - name: b4
      inertial:
        mass: 1
        frame:
          xyz: [0.5, 0, 0]
          rpy: [0, 0, 0]
        inertia:
          Ixx: 0.1
          Iyy: 0.05
          Izz: 0.001
          Iyz: 0
          Ixz: 0
          Ixy: 0
  joints:
    - name: j0
      parent: b0
      child: b1
      type: revolute
      axis: [1, 0, 0]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j1
      parent: b1
      child: b2
      type: revolute
      axis: [0, 1, 0]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j2
      parent: b2
      child: b3
      type: revolute
      axis: [0, 0, 1]
      frame:
        xyz: [0, 1, 0]
        rpy: [0, 0, 0]
      limits:
        upper: 1
        lower: -1
        velocity: 10
        effort: 50
    - name: j3
      parent: b1
      child: b4
      type: continuous
      axis: [1, 0, 0]
      frame:
        xyz: [1, 0, 0]
        rpy: [1, 0, 0]
        anglesInDegrees: false
"""


class TestRBDynParsers(unittest.TestCase):
    def check_result(self, parser_result):
        self.assertEqual(parser_result.name, b"XYZSarm")
        self.assertEqual(len(parser_result.visual), 1)
        self.assertEqual(len(parser_result.collision), 0)
        self.assertEqual(len(parser_result.limits.lower), 4)
        self.assertEqual(len(parser_result.limits.upper), 4)
        self.assertEqual(len(parser_result.limits.velocity), 4)
        self.assertEqual(len(parser_result.limits.torque), 4)
        self.assertEqual(parser_result.mb.nrBodies(), 5)
        self.assertEqual(parser_result.mb.nrJoints(), 5)
        self.assertEqual(parser_result.mbg.nrNodes(), 5)
        self.assertEqual(parser_result.mbg.nrJoints(), 4)

    def test(self):
        self.check_result(rbdyn.parsers.from_urdf(urdf_model))
        self.check_result(rbdyn.parsers.from_yaml(yaml_model))
