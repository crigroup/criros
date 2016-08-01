#! /usr/bin/env python
import criros
import itertools
import numpy as np
import openravepy as orpy

# Convenient variables
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D

def compute_bounding_box_corners(body, Tbody=None, scale=1.0):
  """
  Computes the bounding box corners (8 corners) for the given body.
  If C{Tbody} is given (not None), the corners are transformed.
  The {scale} parameters is a factor used to scale the extents of the
  bounding box.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @type  Tbody: np.array
  @param Tbody: homogeneous transformation to transform the corners. If None,
  the corners are given using the current position of the body in OpenRAVE.
  @type  scale: factor
  @param scale: the scale factor to modify the extents of the bounding box.
  @rtype: list
  @return: list containing the 8 box corners. Each corner is a XYZ point of type C{np.array}.
  """
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

def remove_bodies(env, remove=None, keep=None):
  """
  Removes the specified bodies from the OpenRAVE environment.
  You can specify the bodies to be removed or kept.
  @type  env: orpy.Environment
  @param env: The OpenRAVE environment
  @type  remove: list
  @param remove: list of objects to remove
  @type  keep: list
  @param keep: list of objects to keep 
  """
  # Check that one of the lists is None
  if (remove is None) and (type(keep) is list):
    case = 1
  elif (keep is None) and (type(remove) is list):
    case = 2
  else:
    return
  for body in env.GetBodies():
    remove_body = False
    name = body.GetName()
    if case == 1:
      remove_body = name not in keep
    if case == 2:
      remove_body = name in remove
    if remove_body:
      with env:
        env.Remove(body)

def move_origin_to_body(refbody):
  """
  Moves everything in the OpenRAVE scene so that the C{refbody} ends-up at the origin.
  @type  refbody: orpy.KinBody
  @param refbody: The body that will be at the origin
  """
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

def set_body_transparency(body, transparency=0.0):
  """
  Sets the transparency value of a body recursively.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @type  transparency: float
  @param transparency: The transparency value. If it's out of range [0.0, 1.0], it'll be clipped.
  """
  transparency = np.clip(transparency, 0.0, 1.0)
  env = body.GetEnv()
  with env:
    for link in body.GetLinks():
      for geom in link.GetGeometries():
        geom.SetTransparency(transparency)
