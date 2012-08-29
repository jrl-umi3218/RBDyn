import time
from multiprocessing import Process, Manager, Array, Value, Condition

import numpy as np

from mayavi.tools.animator import Animator

from eigen3 import *
import rbdyn as rbd
import spacevecalg as sva

from task.tasks import PositionTask, PostureTask
from task.solver import WLSSolver

from graph.multibody import GraphicMultiBody
from graph.geometry import MeshGeometry

from robots.little_human import make_little_human

import nethoap


timeLog = []
qLog = []






class Controller(object):
  def __init__(self, mb, mbc, robot):
    self.mb = rbd.MultiBody(mb)
    self.mbcD = rbd.MultiBodyConfig(mbc)
    self.mbcR = rbd.MultiBodyConfig(mbc)

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
    self.mbcR.q = rbd.vectorToParam(self.mb, q)
    self.mbcD.q = rbd.vectorToParam(self.mb, q)

    # update real config
    rbd.forwardKinematics(self.mb, self.mbcR)
    rbd.forwardVelocity(self.mb, self.mbcR)

    # update desired config
    rbd.forwardKinematics(self.mb, self.mbcD)
    rbd.forwardVelocity(self.mb, self.mbcD)

    # compute hand pos objective
    rhandPos = self.mb.bodyIndexById(10)
    pos = list(self.mbcR.bodyPosW)[rhandPos].translation()
    obj = Vector3d(pos[0], pos[1] - 0.10, pos[2])

    posTask = PositionTask(self.mb, 10, obj)
    postureTask = PostureTask(self.mb, q)

    self.postTask = self.solver.addTask(posTask, 100.)
    self.postureTask = self.solver.addTask(postureTask, 1.)


  def run(self):
    # compute next desired position
    self.solver.solve(self.mb, self.mbcD)
    rbd.eulerIntegration(self.mb, self.mbcD, 0.005)

    # control the robot
    self.robot.control(toNumpy(rbd.paramToVector(self.mb, self.mbcD.q)))

    # read sensors
    q = toEigenX(self.robot.sensor())

    # update desired config
    # self.mbcD.q = rbd.vectorToParam(self.mb, q)
    rbd.forwardKinematics(self.mb, self.mbcD)
    rbd.forwardVelocity(self.mb, self.mbcD)

    # update real config
    self.mbcR.q = rbd.vectorToParam(self.mb, q)
    rbd.forwardKinematics(self.mb, self.mbcR)
    rbd.forwardVelocity(self.mb, self.mbcR)



def controllerProcess(ns, startCond, started, qDes, qReal):
  mb, mbc, mbg, objfile = make_little_human('../../robots/hoap_3/mesh/')
  HOST = '10.59.145.197'
  PORT = 55000

  #net = nethoap.NetHoap3(HOST, PORT, mb)
  net = nethoap.FakeHoap3(HOST, PORT, mb)
  cont = Controller(mb, mbc, net)

  startCond.acquire()
  if not started.value:
    startCond.wait()
  startCond.release()

  cont.begin()

  while started.value:
    cmpTimeDeb = time.time()
    cont.run()

    with qDes.get_lock():
      qDesData = np.frombuffer(qDes.get_obj())
      qRealData = np.frombuffer(qReal.get_obj())

      qDesData[:] = toNumpy(rbd.paramToVector(cont.mb, cont.mbcD.q)).T
      qRealData[:] = toNumpy(rbd.paramToVector(cont.mb, cont.mbcR.q)).T



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
    qDes = np.mat(np.zeros((21,1)))
    qReal = np.mat(np.zeros((21,1)))

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
  qDes = Array('d', 21)
  qReal = Array('d', 21)

  # namespace.started = False
  # namespace.qDes = [0.]*21
  # namespace.qReal = [0.]*21

  startCond = Condition()

  contProc = Process(target=controllerProcess, args=(namespace, startCond, started, qDes, qReal))
  contProc.start()

  mb, mbc, mbg, objfile = make_little_human('../../robots/hoap_3/mesh/')
  geomReal = MeshGeometry(mb, objfile)
  geomDes = MeshGeometry(mb, objfile, 0.5)
  graph = Displayer(mb, mbc, geomReal, geomDes, qDes, qReal)

  a = Animator(33, graph.run().next)
  a.timer.Stop()


  def start():
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

