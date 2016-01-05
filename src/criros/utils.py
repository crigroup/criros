#! /usr/bin/env python
import rospy, sys, inspect


def read_parameter(name, default):
  """
  Get a parameter from the ROS parameter server. If it's not found, a 
  warn is printed.
  @type name: string
  @param name: Parameter name
  @param default: Default value for the parameter. The type should be 
  the same as the one expected for the parameter.
  @return: The restulting parameter
  """
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)

def raise_not_defined():
  print 'Method not implemented: %s' % inspect.stack()[1][3]
  sys.exit(1)

def read_parameter_err(name):
  """
  Get a parameter from the ROS parameter server. If it's not found, a 
  error is printed.
  @type name: string
  @param name: Parameter name
  @rtype: has_param, param
  @return: (has_param) True if succeeded, false otherwise. The 
  parameter is None if C{has_param=False}.
  """
  has_param = True
  if not rospy.has_param(name):
    rospy.logerr("Parameter [%s] not found" % (name))
    has_param = False
  return has_param, rospy.get_param(name, None)

def has_keys(data, keys):
  has_all = True
  for key in keys:
    if key not in data:
      has_all = False
      break
  return has_all
