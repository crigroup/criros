#! /usr/bin/env python
import itertools
import numpy as np
import openravepy as orpy
# Image geometry
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
# Transformations
import tf.transformations as tr


def camera_fov_corners(camera_info, zdist, Tcamera=np.eye(4)):
  """
  Generates the 5 corners of the camera field of view
  @type  camera_info: sensor_msgs.CameraInfo
  @param camera_info: Message with the meta information for a camera
  @type  zdist:       float
  @param zdist:       distance from the camera ref frame in the z direction
  @type  Tcamera:     np.array
  @param Tcamera:     homogeneous transformation for the camera ref frame
  @rtype: list
  @return: The 5 corners of the camera field of view
  """
  cam_model = PinholeCameraModel()
  cam_model.fromCameraInfo(camera_info)
  delta_x = cam_model.getDeltaX(cam_model.width/2, zdist)
  delta_y = cam_model.getDeltaY(cam_model.height/2, zdist)
  corners = [Tcamera[:3,3]]
  for k in itertools.product([-1,1],[-1,1]):
    point = np.array([0, 0, zdist, 1])
    point[:2] = np.array([delta_x, delta_y]) * np.array(k)
    corners.append( np.dot(Tcamera, point)[:3] )
  return np.array(corners)

def random_lookat_ray(goal, radius, variance, fov):
  """
  This function returns a random point and the direction from the point to the given goal.
  The random point is inside a chopped upper sphere.
  @type goal      : np.array
  @param goal     : homogeneous transformation of the goal to look at
  @type radius    : float
  @param radius   : minimum radius of the sphere in meters
  @type variance  : float
  @param variance : variance of the radius in meters
  @rtype: orpy.Ray
  @return: The random Ray that looks towards the goal
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

def random_point_inside_fov(camera_info, maxdist, mindist=0, Tcamera=np.eye(4)):
  """
  Generates a random XYZ point inside the camera field of view
  @type  camera_info: sensor_msgs.CameraInfo
  @param camera_info: Message with the meta information for a camera
  @type  maxdist:     float
  @param maxdist:     distance from the camera ref frame in the z direction
  @type  mindist:     float
  @param mindist:     distance from the camera ref frame in the z direction
  @type  Tcamera:     np.array
  @param Tcamera:     homogeneous transformation for the camera ref frame
  @rtype: array
  @return: The random XYZ point
  """
  cam_model = PinholeCameraModel()
  cam_model.fromCameraInfo(camera_info)
  z = np.random.uniform(mindist,maxdist)
  delta_x = cam_model.getDeltaX(cam_model.width/2, z)
  delta_y = cam_model.getDeltaY(cam_model.height/2, z)
  point = np.array([0, 0, z, 1])
  point[:2] = np.array([delta_x, delta_y]) * (2*np.random.random_sample(2) - 1.)
  return np.dot(Tcamera, point)[:3]
