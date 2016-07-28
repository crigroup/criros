#! /usr/bin/env python
import criros
import itertools
import numpy as np
import openravepy as orpy


def compute_bounding_box_corners(body, Tbody=None, scale=1.0):
  # Create a dummy body an use OpenRAVE to get the corners
  env = body.GetEnv()
  dummy = orpy.RaveCreateKinBody(env, '')
  dummy.Clone(body, 0)
  if Tbody is not None:
    with env:
      dummy.SetTransform(Tbody)
  aabb = dummy.ComputeAABB()
  corners = []
  for k in itertools.product([-1,1],[-1,1],[-1,1]):
    corners.append(aabb.pos() + np.array(k)*aabb.extents()*scale)
  return corners

def remove_objects(env, objects):
  for name in objects:
    body =  env.GetKinBody(name)
    if body is not None:
      env.Remove(body)

def move_origin_to_body(refbody):
  env = refbody.GetEnv()
  Toffset = criros.spalg.transform_inv( refbody.GetTransform() )
  grabbed_names = [body.GetName() for robot in env.GetRobots() for body in robot.GetGrabbed()]
  with env:
    for body in env.GetBodies():
      # Dont move Grabbed bodies. They will move once we move the robot grabbing them.
      if body.GetName in grabbed_names:
        continue
      Tbody = body.GetTransform()
      body.SetTransform( np.dot(Tbody, Toffset) )
