import time

import socket
import select
from threading import Thread, Lock
from struct import unpack

import numpy as np

from eigen3 import *
import rbdyn as rbd

from graph.multibody import GraphicMultiBody
from graph.geometry import DefaultGeometry

from robots.little_human import make_little_human

from mayavi.tools.animator import Animator

from task.tasks import PositionTask, PostureTask
from task.solver import WLSSolver


thetaToPulse = np.array([209, 209, -209, 209, 209, -209, 209, 209, -209, -209,
                         209, 209, 209, -209, -209, -209, -209, 209, -209, 209,
                         209])

pulseToTheta = 1./thetaToPulse


class Network(Thread):
  def __init__(self, host, port, mb, cont, graph):
    super(Network, self).__init__()

    self.host = host
    self.port = port

    self.mb = mb

    self.cont = cont
    self.graph = graph

    self.index = np.array([mb.jointPosInParam(mb.jointIndexById(i)) for i in range(1, 22)])

    self.mbcLock = Lock()
    self.q = None

    self.isData = False


  def run(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print 'Connected'

    p = select.poll()
    p.register(s, select.POLLIN)

    while True:
      p.poll()
      data = s.recv(70)

      if data == None or len(data) != 70:
        print 'Connexion broken'
        return

      pyData = unpack('21h8h6h', data)

      q = np.array(pyData[:21])*pulseToTheta
      qn = q.copy()
      qn[self.index] = q
      qn = np.mat(qn).T
      qn = np.deg2rad(qn)
      self.qn = qn

      with self.mbcLock:
        self.q = rbd.vectorToParam(self.mb, toEigen(qn))

      if not self.isData:
        self.isData = True

        self.cont.initTask(self.q)
        self.cont.start()
        self.graph.timer.Start()



class Controller(Thread):
  def __init__(self, mb, mbc):
    super(Controller, self).__init__()

    self.mb = rbd.MultiBody(mb)
    self.mbc = rbd.MultiBodyConfig(mbc)

    self.solver = WLSSolver()

    self.net = None


  def initTask(self, q):
    self.mbc.q = q

    rbd.forwardKinematics(self.mb, self.mbc)
    rhandPos = self.mb.bodyIndexById(10)
    pos = list(self.mbc.bodyPosW)[rhandPos].translation()
    obj = Vector3d(pos[0] + 0.10, pos[1] + 0.10, pos[2])

    posTask = PositionTask(self.mb, 10, obj)
    postureTask = PostureTask(self.mb, VectorXd.Zero(mb.nrDof()))

    self.postTask = self.solver.addTask(posTask, 100.)
    self.postureTask = self.solver.addTask(postureTask, 1.)


  def run(self):
    while self.net.is_alive():
      with self.net.mbcLock:
        self.mbc.q = self.net.q

      rbd.forwardKinematics(self.mb, self.mbc)
      rbd.forwardVelocity(self.mb, self.mbc)

      self.solver.solve(self.mb, self.mbc)
      rbd.eulerIntegration(self.mb, self.mbc, 0.001)

      time.sleep(0.001)




class Displayer(object):
  def __init__(self, mb, mbc, geom):
    self.mb = rbd.MultiBody(mb)
    self.mbc = rbd.MultiBodyConfig(mbc)
    self.net = None

    rbd.forwardKinematics(mb, mbc)

    self.graph = GraphicMultiBody(mb, geom)
    self.graph.draw(self.mb, self.mbc)
    self.graph.render()



  def run(self):
    while True:
      with self.net.mbcLock:
        self.mbc.q = self.net.q

      rbd.forwardKinematics(self.mb, self.mbc)

      self.graph.draw(self.mb, self.mbc)
      self.graph.render()
      yield



if __name__ == '__main__':
  mb, mbc, mbg = make_little_human()
  geom = DefaultGeometry(mb)

  HOST = '10.59.145.197'
  #HOST = 'localhost'
  PORT = 55000

  graph = Displayer(mb, mbc, geom)
  cont = Controller(mb, mbc)

  a = Animator(33, graph.run().next)
  a.timer.Stop()

  net = Network(HOST, PORT, mb, cont, a)

  graph.net = net
  cont.net = net

  def start():
    net.start()

  def stop():
    net.stop()
    a.timer.stop()
    cont.stop()

  #a.edit_traits()

