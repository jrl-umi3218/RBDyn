import robots.amelif as amelif


if __name__ == '__main__':
  mb, mbc, mbg, objfile = amelif.from_amelif('../../robots/hrp2_10/xml/hrp2_10-small.xml', True)

  bound = dict([(j.id(), (0., 0.)) for j in mb.joints()])

  doc = amelif.to_amelif(mb, bound, objfile)
  print doc.toprettyxml(indent='  ')


