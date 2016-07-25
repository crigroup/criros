#! /usr/bin/env python
import numpy as np
import openravepy as orpy
import tf.transformations as tr


def random_lookat_ray(goal, radius, variance, fov):
  """
  This function returns a random point and the direction from the point to the pattern position.
  The random point is on a chopped upper sphere of the point.
  @type goal      : np.array
  @param goal     : homogeneous transformation of the goal to look at
  @type radius    : float
  @param radius   : minimum radius of the sphere in meters
  @type variance  : float
  @param variance : variance of the radius in meters
  """
  theta1 = 2.*np.pi*np.random.uniform(-fov, fov)
  theta2 = np.arccos(1 - np.random.uniform(0, fov)**2)
  r = radius + variance*np.random.uniform(0,1.)
  x = r*np.cos(theta1)*np.sin(theta2)
  y = r*np.sin(theta1)*np.sin(theta2)
  z = r*np.cos(theta2)
  R = goal[:3,:3]
  point = goal[:3,3] + np.dot(R, np.array([x,y,z]))
  # Find the direction
  direction = -np.dot(R, np.array([x,y,z]))
  direction = tr.unit_vector(direction)
  return orpy.Ray(point, direction)
