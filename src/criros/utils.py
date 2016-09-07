#! /usr/bin/env python
import numpy as np
import rospy, sys, inspect


class TextColors:
  """
  The C{TextColors} class is used as alternative to the C{rospy} logger. It's useful to
  print messages when C{roscore} is not running.
  """
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  
  def disable(self):
    """
    Resets the coloring.
    """
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''
  
  def blue(self, msg):
    """
    Prints a B{blue} color message
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.OKBLUE + msg + self.ENDC)
  
  def debug(self, msg):
    """
    Prints a B{green} color message
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.OKGREEN + msg + self.ENDC)
  
  def error(self, msg):
    """
    Prints a B{red} color message
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.FAIL + msg + self.ENDC)
  
  def ok(self, msg):
    """
    Prints a B{green} color message
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.OKGREEN + msg + self.ENDC)
  
  def warning(self, msg):
    """
    Prints a B{yellow} color message
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.WARNING + msg + self.ENDC)
  
  def logdebug(self, msg):
    """
    Prints message with the word 'Debug' in green at the begging. 
    Alternative to C{rospy.logdebug}.
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.OKGREEN + 'Debug ' + self.ENDC + str(msg))
    
  def loginfo(self, msg):
    """
    Prints message with the word 'INFO' begging. 
    Alternative to C{rospy.loginfo}.
    @type  msg: string
    @param msg: the message to be printed.
    """
    print('INFO ' + str(msg))
  
  def logwarn(self, msg):
    """
    Prints message with the word 'Warning' in yellow at the begging. 
    Alternative to C{rospy.logwarn}.
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.WARNING + 'Warning ' + self.ENDC + str(msg))
  
  def logerr(self, msg):
    """
    Prints message with the word 'Error' in red at the begging. 
    Alternative to C{rospy.logerr}.
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.FAIL + 'Error ' + self.ENDC + str(msg))
  
  def logfatal(self, msg):
    """
    Prints message with the word 'Fatal' in red at the begging. 
    Alternative to C{rospy.logfatal}.
    @type  msg: string
    @param msg: the message to be printed.
    """
    print(self.FAIL + 'Fatal ' + self.ENDC + str(msg))


## Helper Functions ##
def assert_shape(variable, name, shape):
  """
  Asserts the shape of an np.array
  @type  variable: Object
  @param variable: variable to be asserted
  @type  name: string
  @param name: variable name
  @type  shape: tuple
  @param ttype: expected shape of the np.array
  """
  assert variable.shape == shape, '%s must have a shape %r: %r' % (name, shape, variable.shape)
  
def assert_type(variable, name, ttype):
  """
  Asserts the type of a variable with a given name
  @type  variable: Object
  @param variable: variable to be asserted
  @type  name: string
  @param name: variable name
  @type  ttype: Type
  @param ttype: expected variable type
  """
  assert type(variable) is ttype,  '%s must be of type %r: %r' % (name, ttype, type(variable))

def db_error_msg(name, logger=TextColors()):
  """
  Prints out an error message appending the given database name.
  @type  name: string
  @param name: database name
  @type  logger: Object
  @param logger: Logger instance. When used in ROS, the recommended C{logger=rospy}.
  """
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
  """
  Checks whether a dictionary has all the given keys.
  @type   data: dict
  @param  data: Parameter name
  @type   keys: list
  @param  keys: list containing the expected keys to be found in the dict.
  @rtype: bool
  @return: True if all the keys are found in the dict, false otherwise.
  """
  if not isinstance(data, dict):
    return False
  has_all = True
  for key in keys:
    if key not in data:
      has_all = False
      break
  return has_all

def raise_not_implemented():
  """
  Prints an 'method not implemented' msg and exits with error code 1.
  """
  print 'Method not implemented: %s' % inspect.stack()[1][3]
  sys.exit(1)

def read_parameter(name, default):
  """
  Get a parameter from the ROS parameter server. If it's not found, a 
  warn is printed.
  @type  name: string
  @param name: Parameter name
  @type  default: Object
  @param default: Default value for the parameter. The type should be 
  the same as the one expected for the parameter.
  @rtype: any
  @return: The resulting parameter
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
    logger.logerr('roscore not found')
    has_param = False
  else:
    has_param = True
    if not rospy.has_param(name):
      rospy.logerr("Parameter [%s] not found" % (name))
      has_param = False
  return has_param, rospy.get_param(name, None)

def read_parameter_fatal(name):
  """
  Get a parameter from the ROS parameter server. If it's not found, an
  exception will be raised.
  @type name: string
  @param name: Parameter name
  @rtype: any
  @return: The resulting parameter
  """
  if rospy.is_shutdown():
    logger = TextColors()
    logger.logfatal('roscore not found')
    raise Exception( 'Required parameter {0} not found'.format(name) )
  else:
    if not rospy.has_param(name):
      rospy.logfatal("Parameter [%s] not found" % (name))
      raise Exception( 'Required parameter {0} not found'.format(name) )
  return rospy.get_param(name, None)

def solve_namespace(namespace=''):
  """
  Appends neccessary slashes required for a proper ROS namespace.
  @type namespace: string
  @param namespace: namespace to be fixed.
  @rtype: string
  @return: Proper ROS namespace.
  """
  if len(namespace) == 0:
    namespace = rospy.get_namespace()
  elif len(namespace) == 1:
    if namespace != '/':
      namespace = '/' + namespace + '/'
  else:
    if namespace[0] != '/':
      namespace = '/' + namespace
    if namespace[-1] != '/':
      namespace += '/'
  return namespace

def unique(data):
  """
  Finds the unique elements of an array. B{row-wise} and 
  returns the sorted unique elements of an array.
  @type  data: np.array
  @param data: Input array.
  @rtype: np.array
  @return: The sorted unique array.
  """
  order = np.lexsort(data.T)
  data = data[order]
  diff = np.diff(data, axis=0)
  ui = np.ones(len(data), 'bool')
  ui[1:] = (diff != 0).any(axis=1) 
  return data[ui]
