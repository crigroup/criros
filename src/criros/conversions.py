#! /usr/bin/env python
import rospy
import PyKDL
import numpy as np


# PyKDL types <--> Numpy types
def from_kdl_vector(vector):
  """
  Converts a C{PyKDL.Vector} with fields into a numpy array.
  
  @type vector: PyKDL.Vector
  @param vector: The C{PyKDL.Vector} to be converted
  @rtype: array
  @return: The resulting numpy array
  """
  return np.array([vector.x(),vector.y(),vector.z()])
  
def from_kdl_twist(twist):
  """
  Converts a C{PyKDL.Twist} with fields into a numpy array.
  
  @type twist: PyKDL.Twist
  @param twist: The C{PyKDL.Twist} to be converted
  @rtype: array
  @return: The resulting numpy array
  """
  array = np.zeros(6)
  array[:3] = from_kdl_vector(twist.vel)
  array[3:] = from_kdl_vector(twist.rot)
  return array
  
  
# ROS types <--> Numpy types
def from_point(msg):
  """
  Converts a C{geometry_msgs/Point} ROS message into a numpy array.
  
  @type msg: geometry_msgs/Point
  @param msg: The ROS message to be converted
  @rtype: array
  @return: The resulting numpy array
  """
  return np.array([msg.x, msg.y, msg.z])
  
def from_vector3(msg):
  """
  Converts a C{geometry_msgs/Vector3} ROS message into a numpy array.
  
  @type msg: geometry_msgs/Vector3
  @param msg: The ROS message to be converted
  @rtype: array
  @return: The resulting numpy array
  """
  return np.array([msg.x, msg.y, msg.z])
  
def from_wrench(msg):
  """
  Converts a C{geometry_msgs/Wrench} ROS message into a numpy array.
  
  @type msg: geometry_msgs/Wrench
  @param msg: The ROS message to be converted
  @rtype: array
  @return: The resulting numpy array
  """
  array = np.zeros(6)
  array[:3] = from_vector3(msg.force)
  array[3:] = from_vector3(msg.torque)
  return array
