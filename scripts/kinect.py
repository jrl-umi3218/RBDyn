#!/usr/bin/python
## The equivalent of:
##  "Working with the Skeleton"
## in the OpenNI user guide.

"""
This shows how to identify when a new user is detected, look for a pose for
that user, calibrate the users when they are in the pose, and track them.

Specifically, it prints out the location of the users' head,
as they are tracked.
"""

from multiprocessing import Array
from openni import *

handPos = Array('d', 3)

def kinect():
  context = Context()
  context.init()

  depth_generator = DepthGenerator()
  depth_generator.create(context)
  depth_generator.set_resolution_preset(RES_VGA)
  depth_generator.fps = 30

  gesture_generator = GestureGenerator()
  gesture_generator.create(context)
  gesture_generator.add_gesture('Wave')

  hands_generator = HandsGenerator()
  hands_generator.create(context)
  hands_generator.set_smoothing(0.2)

  # Declare the callbacks
  # gesture
  def gesture_detected(src, gesture, id, end_point):
    print "Detected gesture:", gesture
    hands_generator.start_tracking(end_point)

  def gesture_progress(src, gesture, point, progress):
    pass

  def create(src, id, pos, time):
    print 'Create ', id, pos

  def update(src, id, pos, time):
    handPos[:] = pos

  def destroy(src, id, time):
    print 'Destroy ', id

  # Register the callbacks
  gesture_generator.register_gesture_cb(gesture_detected, gesture_progress)
  hands_generator.register_hand_cb(create, update, destroy)

  # Start generating
  context.start_generating_all()

  print 'Make a Wave to start tracking...'

  while True:
    context.wait_any_update_all()
