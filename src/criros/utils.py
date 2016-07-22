#! /usr/bin/env python
import rospy, sys, inspect


class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  
  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''
  
  def blue(self, msg):
    print(self.OKBLUE + msg + self.ENDC)
  
  def debug(self, msg):
    print(self.OKGREEN + msg + self.ENDC)
  
  def error(self, msg):
    print(self.FAIL + msg + self.ENDC)
  
  def ok(self, msg):
    print(self.OKGREEN + msg + self.ENDC)
  
  def warning(self, msg):
    print(self.WARNING + msg + self.ENDC)
  
  def logdebug(self, msg):
    print(self.OKGREEN + 'Debug ' + self.ENDC + str(msg))
    
  def loginfo(self, msg):
    print('INFO ' + str(msg))
  
  def logwarn(self, msg):
    print(self.WARNING + 'Warning ' + self.ENDC + str(msg))
  
  def logerr(self, msg):
    print(self.FAIL + 'Error ' + self.ENDC + str(msg))
  
  def logfatal(self, msg):
    print(self.FAIL + 'Fatal ' + self.ENDC + str(msg))


## Helper Functions ##
def assert_shape(variable, name, shape):
  assert variable.shape == shape, '%s must have a shape %r: %r' % (name, shape, variable.shape)
  
def assert_type(variable, name, ttype):
  assert type(variable) is ttype,  '%s must be of type %r: %r' % (name, ttype, type(variable))

def db_error_msg(name, logger=TextColors()):
  msg='Database %s not found. Please generate it. [rosrun denso_openrave generate_databases.py]' % name
  logger.logerr(msg)

def clean_cos(value):
  """
  Limits the a value between the range C{[-1, 1]}
  @type value: float
  @param value: The input value
  @rtype: float
  @return: The limited value in the range C{[-1, 1]}
  """
  return min(1,max(value,-1))

def has_keys(data, keys):
  has_all = True
  for key in keys:
    if key not in data:
      has_all = False
      break
  return has_all

def raise_not_implemented():
  print 'Method not implemented: %s' % inspect.stack()[1][3]
  sys.exit(1)

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
  if rospy.is_shutdown():
    logger = TextColors()
    logger.logwarn('roscore not found, parameter [%s] using default: %s' % (name, default))
  else:
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
  return default

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
  if rospy.is_shutdown():
    logger = TextColors()
    logger.logwarn('roscore not found, parameter [%s] using default: %s' % (name, default))
    has_param = False
  else:
    has_param = True
    if not rospy.has_param(name):
      rospy.logerr("Parameter [%s] not found" % (name))
      has_param = False
  return has_param, rospy.get_param(name, None)

def unique(a):
  """
  Finds the unique elements of an array. B{row-wise} and 
  returns the sorted unique elements of an array.
  
  @type a: numpy.array
  @param a: Input array.
  @rtype: numpy.array
  @return: The sorted unique array.
  """
  order = np.lexsort(a.T)
  a = a[order]
  diff = np.diff(a, axis=0)
  ui = np.ones(len(a), 'bool')
  ui[1:] = (diff != 0).any(axis=1) 
  return a[ui]
