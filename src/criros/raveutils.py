#! /usr/bin/env python
import criros
import numpy as np
import openravepy as orpy

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
