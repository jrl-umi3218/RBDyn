import os

import robots.little_human as hoap

from robots.amelif import to_amelif


if __name__ == '__main__':
  mb, mbc, mbg = hoap.robot()
  boundVec = hoap.bound(mb)
  objfile, stpbvfiles, convexfiles = hoap.ressources('')

  boundDict = dict([(mb.joint(i).id(), (l[0], u[0], tl[0], tu[0])) for i, (l, u, tl, tu) in enumerate(zip(*boundVec)) if len(l) > 0])

  wrlfile = dict([(id, ['afresources' + os.sep + 'hoap3' + os.sep + 'vrml' + os.sep +
                   file[0].split(os.sep)[-1].replace('.obj', '.wrl')]) for id, file in objfile.iteritems()])

  doc = to_amelif(mb, boundDict, wrlfile, tuple([0.001]*3))
  print doc.toprettyxml(indent='  ')

