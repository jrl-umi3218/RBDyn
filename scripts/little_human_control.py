import time
from multiprocessing import Process, Manager, Array, Value, Condition

import numpy as np

from mayavi.tools.animator import Animator

from eigen3 import *
import rbdyn as rbd
import spacevecalg as sva

from task.tasks import PositionTask, PostureTask, CoMTask, NoRotationTask
from task.solver import WLSSolver

from graph.multibody import GraphicMultiBody
from graph.geometry import MeshGeometry, DefaultGeometry

from robots.little_human import make_little_human

import nethoap


timeLog = []
qLog = []






class Controller(object):
  def __init__(self, mb, mbc, mbg, robot):
    self.mbReal = rbd.MultiBody(mb)

    r = np.mat([[0., 0., 1.],
                [0., -1., 0.],
                [1., 0., 0.]])
    tr = sva.PTransform(toEigen(r))
    self.mbRF = mbg.makeMultiBody(6, True, tr)
    self.mbLF = mbg.makeMultiBody(16, True, tr)
    self.mbPlan = self.mbRF

    self.mbcReal = rbd.MultiBodyConfig(mbc)
    self.mbcControl = rbd.MultiBodyConfig(mbc)

    self.mbcRF = rbd.MultiBodyConfig(self.mbRF)
    self.mbcRF = rbd.MultiBodyConfig(self.mbLF)
    self.mbcPlan = self.mbcRF

    rfPos = self.mbReal.bodyIndexById(22)
    lfPos = self.mbReal.bodyIndexById(22)

    self.rfToWaist = self.mbReal.transform(rfPos).inv()
    self.lfToWaist = self.mbReal.transform(lfPos).inv()
    self.planToWaist = self.rfToWaist

    self.rfWaistPos = self.mbRF.bodyIndexById(21)
    self.lfWaistPos = self.mbLF.bodyIndexById(21)
    self.planWaistPos = self.rfWaistPos

    self.realToR = rbd.ConfigConverter(self.mbReal, self.mbRF).convert
    self.realToL = rbd.ConfigConverter(self.mbReal, self.mbLF).convert
    self.realToPlan = self.realToR

    self.rToReal = rbd.ConfigConverter(self.mbRF, self.mbReal).convert
    self.lToReal = rbd.ConfigConverter(self.mbLF, self.mbReal).convert
    self.planToReal = self.rToReal

    self.solver = WLSSolver()

    self.robot = robot


  def begin(self):
    self.robot.open()
    self.initTask()


  def end(self):
    self.robot.close()


  def initTask(self):
    # take current q
    q = toEigenX(self.robot.sensor())
    self.mbcReal.q = rbd.vectorToParam(self.mbReal, q)
    self.realToPlan(self.mbcReal, self.mbcPlan);

    # update desired config
    rbd.forwardKinematics(self.mbPlan, self.mbcPlan)
    rbd.forwardVelocity(self.mbPlan, self.mbcPlan)

    # compute foot pos objective
    lFootInd = self.mbPlan.bodyIndexById(16)
    rFootInd = self.mbPlan.bodyIndexById(6)
    self.lFootPos = list(self.mbcPlan.bodyPosW)[lFootInd].translation()
    self.rFootPos = list(self.mbcPlan.bodyPosW)[rFootInd].translation()
    obj = Vector3d(self.lFootPos[0], self.lFootPos[1], self.lFootPos[2])

    # compute CoM pos objective
    comObj = rbd.computeCoM(self.mbPlan, self.mbcPlan)
    comObj = Vector3d(comObj[0], comObj[1] + 0.01, comObj[2])
    comObj = Vector3d(self.lFootPos[0], self.lFootPos[1], comObj[2])
    self.com = 'l'

    posTask = PositionTask(self.mbPlan, 16, obj)
    noRotTask = NoRotationTask(self.mbPlan, 16)
    postureTask = PostureTask(self.mbPlan, rbd.paramToVector(self.mbPlan, self.mbcPlan.q))
    comTask = CoMTask(self.mbPlan, comObj)

    self.posTask = self.solver.addTask(posTask, 1000000.)
    self.noRotTask = self.solver.addTask(noRotTask, 1000000.)
    self.postureTask = self.solver.addTask(postureTask, 10.)
    self.comTask = self.solver.addTask(comTask, 100.)


  def run(self):
    # compute next desired position
    self.solver.solve(self.mbPlan, self.mbcPlan)
    rbd.eulerIntegration(self.mbPlan, self.mbcPlan, 0.005)


    if np.linalg.norm(self.comTask[0].error()) < 0.025:
      if self.com == 'l':
        self.comTask[0].obj[0] = self.rFootPos[0]
        self.comTask[0].obj[1] = self.rFootPos[1]
        self.com = 'r'
      else:
        self.comTask[0].obj[0] = self.lFootPos[0]
        self.comTask[0].obj[1] = self.lFootPos[1]
        self.com = 'l'


    self.planToReal(self.mbcPlan, self.mbcControl);

    # control the robot
    self.robot.control(toNumpy(rbd.paramToVector(self.mbReal, self.mbcControl.q)))

    # read sensors
    q = toEigenX(self.robot.sensor())

    # update real config
    self.mbcReal.q = rbd.vectorToParam(self.mbReal, q)

    # update desired config
    # self.realToPlan(self.mbcReal, self.mbcPlan);
    rbd.forwardKinematics(self.mbPlan, self.mbcPlan)
    rbd.forwardVelocity(self.mbPlan, self.mbcPlan)



def controllerProcess(ns, startCond, started, qDes, qReal):
  mb, mbc, mbg, objfile = make_little_human('../../robots/hoap_3/mesh/')
  HOST = '10.59.145.197'
  PORT = 55000

  net = nethoap.NetHoap3(HOST, PORT, mb)
  # net = nethoap.FakeHoap3(HOST, PORT, mb)
  cont = Controller(mb, mbc, mbg, net)

  startCond.acquire()
  if not started.value:
    startCond.wait()
  startCond.release()

  cont.begin()

  while started.value:
    cmpTimeDeb = time.time()
    cont.run()

    waist = cont.planToWaist*list(cont.mbcPlan.bodyPosW)[cont.planWaistPos]
    waistR = waist.rotation()
    waistQ = Quaterniond(waist.rotation()).inverse()
    waistRN = np.mat([waistQ.w(), waistQ.x(), waistQ.y(), waistQ.z()])

    waistT = waist.translation()
    waistN = toNumpy(waistT)
    waistN = toNumpy(waistR*waistT)

    with qDes.get_lock():
      qDesData = np.frombuffer(qDes.get_obj())
      qRealData = np.frombuffer(qReal.get_obj())

      qDesData[:4] = waistRN
      qRealData[:4] = waistRN
      qDesData[4:7] = waistN.T
      qRealData[4:7] = waistN.T
      qDesData[7:] = toNumpy(rbd.paramToVector(cont.mbReal, cont.mbcControl.q)).T
      qRealData[7:] = toNumpy(rbd.paramToVector(cont.mbReal, cont.mbcReal.q)).T



    cmpTimeEnd = time.time()
    ellapsed = cmpTimeEnd - cmpTimeDeb
    ellapsed = min(0.005, ellapsed)

    time.sleep(0.005 - ellapsed)

  cont.end()



class Displayer(object):
  def __init__(self, mb, mbc, geomReal, geomDes, qDes, qReal):
    self.mb = rbd.MultiBody(mb)
    self.mbcReal = rbd.MultiBodyConfig(mbc)
    self.mbcDes = rbd.MultiBodyConfig(mbc)
    self.qDes = qDes
    self.qReal = qReal

    # display hoap3
    rbd.forwardKinematics(self.mb, self.mbcReal)
    rbd.forwardKinematics(self.mb, self.mbcDes)

    self.graphReal = GraphicMultiBody(self.mb, geomReal)
    self.graphDes = GraphicMultiBody(self.mb, geomDes, sva.PTransform(Vector3d(0., 0.5, 0.)))

    self.graphReal.draw(self.mb, self.mbcReal)
    self.graphDes.draw(self.mb, self.mbcDes)

    self.graphReal.render()


  def run(self):
    qDes = np.mat(np.zeros((28,1)))
    qReal = np.mat(np.zeros((28,1)))

    while True:
      with self.qDes.get_lock():
        qDesData = np.frombuffer(self.qDes.get_obj())
        qRealData = np.frombuffer(self.qReal.get_obj())
        qDes[:] = np.mat(qDesData).T
        qReal[:] = np.mat(qRealData).T

      self.mbcReal.q = rbd.vectorToParam(self.mb, toEigenX(qReal))
      self.mbcDes.q = rbd.vectorToParam(self.mb, toEigenX(qDes))

      rbd.forwardKinematics(self.mb, self.mbcReal)
      rbd.forwardKinematics(self.mb, self.mbcDes)

      self.graphReal.draw(self.mb, self.mbcReal)
      self.graphDes.draw(self.mb, self.mbcDes)

      self.graphReal.render()

      yield



if __name__ == '__main__':
  manager = Manager()
  namespace = manager.Namespace()

  started = Value('b', False)
  qDes = Array('d', 28)
  qReal = Array('d', 28)

  qDesData = np.frombuffer(qDes.get_obj())
  qRealData = np.frombuffer(qReal.get_obj())

  qDesData[1:] = np.mat(np.zeros((27,)))
  qRealData[1:] = np.mat(np.zeros((27,)))
  qDesData[0] = 1.
  qRealData[0] = 1.

  startCond = Condition()

  contProc = Process(target=controllerProcess, args=(namespace, startCond, started, qDes, qReal))

  mb, mbc, mbg, objfile = make_little_human('../../robots/hoap_3/mesh/', False)
  geomReal = MeshGeometry(mb, objfile, 0.3)
  # geomDes = MeshGeometry(mb, objfile, 0.5)
  # geomReal = DefaultGeometry(mb)
  geomDes = DefaultGeometry(mb)
  graph = Displayer(mb, mbc, geomReal, geomDes, qDes, qReal)

  a = Animator(33, graph.run().next)
  a.timer.Stop()


  def start():
    contProc.start()

    startCond.acquire()
    started.value = True
    startCond.notify()
    startCond.release()
    a.timer.Start()


  def stop():
    a.timer.Stop()
    started.value = False
    contProc.join()


  #a.edit_traits()

