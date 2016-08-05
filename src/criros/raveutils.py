#! /usr/bin/env python
import copy
import criros
import itertools
import numpy as np
import openravepy as orpy
# ROS
import rospy
import roslib.message
import rosgraph
from sensor_msgs.msg import JointState
# Transformations
import tf
import tf.transformations as tr
# Logger
from criros.utils import TextColors
# Messages
import tabulate
from std_msgs.msg import String

# Convenient variables
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D

class RaveStateUpdater():
  """
  The C{RaveStateUpdater} class connects to ROS C{TF} and updates the given
  OpenRAVE environment. It uses the bodies and robots names available in the
  OpenRAVE scene to find the corresponding information in the C{joint_states}
  topics and in C{TF}.
  """
  def __init__(self, envid, fixed_frame='base_link', rate=10., logger=TextColors()):
    """
    C{RaveStateUpdater} constructor. It uses a C{tf.TransformListener} instance to query TF.
    
    By default, after creating it will start updating the environment at the given
    rate. Every time it updates, check for new frame ids in TF.
    
    The body names and the TF frame ids must have similarities and when possible
    should be the same. We check C{frame_id in body_name} and C{body_namein frame_id}
    this way if C{frame_id='left/base_link'} and C{body_name='denso_left'}, we will 
    split the C{frame_id} into C{['left','base_link']} and will be able to match 
    them.
    
    @type  envid: int
    @param envid: Id of the OpenRAVE environment to be updated
    @type  fixed_frame: string
    @param fixed_frame: Name of the TF frame to be considered as the fixed frame.It is also used to locate the corresponding 
    OpenRAVE body name.
    @type  rate: float
    @param rate: Rate at which we will query TF for new transforms.
    @type  logger: Object
    @param logger: Logger instance. When used in ROS, the recommended C{logger=rospy}.
    """
    self.env = orpy.RaveGetEnvironment(envid)
    try:
      self.listener = tf.TransformListener()
    except rospy.ROSInitException:
      logger.logerr('time is not initialized. Have you called rospy.init_node()?')
      return
    # Wait a little bit until we hear something from tf
    rospy.sleep(1.0)
    # Subscribe to the joint_states topics that match a robot in OpenRAVE
    topics = rosgraph.Master('/rostopic').getPublishedTopics('/')
    self.js_topics = []
    self.js_robots = []
    for topic_info in topics:
      topic_name = topic_info[0]
      msg_class = roslib.message.get_message_class(topic_info[1])
      if ('joint_states' in topic_name) and (msg_class is JointState):
        namespace = topic_name.replace('/', '').replace('joint_states','')
        robot = self._find_rave_body(namespace)
        if robot is not None:
          self.js_topics.append(topic_name)
          self.js_robots.append(robot.GetName())
    self.js_msgs = [None]*len(self.js_topics)
    for topic in self.js_topics:
      rospy.Subscriber(topic, JointState, self._cb_joint_states, callback_args=topic)
    # Look for the body that corresponds to the reference frame
    self.fixed_frame = self._get_tf_reference_frame(fixed_frame)
    if self.fixed_frame is None:
      logger.logerr('Failed to find an unique frame in the TF tree. Found frames: {0}'.format( self.listener.getFrameStrings() ))
      return
    # Make the origin coincide with the reference frame body
    self.ref_body = self._find_rave_body(self.fixed_frame)
    if self.ref_body is None:
      body_names = [body.GetName() for body in self.env.GetBodies()]
      logger.logerr('Failed to find the reference body in OpenRAVE. Found bodies: {0}'.format( body_names ))
      return
    criros.raveutils.move_origin_to_body(self.ref_body)
    # Create a publisher that will report the affected OpenRAVE objects
    self.pub = rospy.Publisher('/openrave/state_updater', String, queue_size=3)
    # Start a thread to update all the objects found in TF and OpenRAVE
    self.rate = rate
    self.elapsed_time = 0.0
    self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.update_rave_environment)
    rospy.on_shutdown(self.stop)
  
  def _cb_joint_states(self, msg, topic):
    """
    Callback that will be executed everytime a message is published in
    the subscribed topics.
    @type  msg: sensor_msgs.msg.JointState
    @param msg: the published C{JointState} message
    @type  topic: string
    @param topic: name of the topic that made the call
    """
    if topic not in self.js_topics:
      return
    idx = self.js_topics.index(topic)
    self.js_msgs[idx] = copy.deepcopy(msg)
  
  def _get_tf_reference_frame(self, name):
    """
    Finds the corresponding frame_id in TF for the given keyword (C{name}).
    @type  name: string
    @param name: name used to search for the frame_id
    @rtype: string
    @return: The existing frame_id
    """
    tf_frame = [frame for frame in self.listener.getFrameStrings() if name in frame]
    if len(tf_frame) != 1:
      return None
    return tf_frame[0]
  
  def _find_rave_body(self, frame_id):
    """
    Finds the corresponding C{orpy.KinBody} in OpenRAVE for the given keyword (C{frame_id}).
    @type  frame_id: string
    @param frame_id: frame_id used to search for the body in OpenRAVE
    @rtype: orpy.KinBody
    @return: Reference to the body in OpenRAVE
    """
    # Remove slashes found at the beggining of the frame_id
    if frame_id[0] == '/':
      frame_id = frame_id[1:]
    ref_body = None
    for possible_name in frame_id.split('/'):
      for body in self.env.GetBodies():
        body_name = body.GetName()
        if (body_name in possible_name) or (possible_name in body_name):
          ref_body = body
          break
      if ref_body is not None:
        break
    return ref_body
  
  def stop(self):
    """
    If the update thread is running, stops it.
    Once you stop the update thread B{you have to call manually} the C{update_rave_environment}.
    """
    if self.timer.is_alive():
      self.timer.shutdown()
  
  def update_rave_environment(self, event=None):
    """
    Reads the all the transformations of the bodies matching some TF frame with respect
    to the C{fixed_frame}, then, updates the OpenRAVE environment.
    Additionally, will read from the joint_states topics and update the robots joint values
    in OpenRAVE.
    @type  event: rospy.timer.TimerEvent
    @param event: Unused. It's required so that the function can be called from a C{rospy.Timer}.
    """
    # Variables used to report the updated states
    updated_bodies = []
    updated_robots = []
    # Added the fixed frame
    ref_body = self._find_rave_body(self.fixed_frame)
    if ref_body is not None:
      updated_bodies.append( [ref_body.GetName(), self.fixed_frame, '* Fixed frame'] )
    # Get available names in TF excluding the fixed_frame
    tf_names = []
    for frame_id in self.listener.getFrameStrings():
      if '/' in frame_id:
        if 'base_link' in frame_id:
          tf_names.append(frame_id)
      else:
        tf_names.append(frame_id)
    tf_names = list( set(tf_names) )
    if self.fixed_frame in tf_names:
      idx = tf_names.index(self.fixed_frame)
      tf_names.pop(idx)
    # Find the corresponding OpenRAVE bodies and update their transform
    timeout = self.rate / (float(len(tf_names))+1)
    for frame_id in tf_names:
      body = self._find_rave_body(frame_id)
      if body is None:
        continue
      try:
        # Access the latest available transforms in the TF tree,
        (pos,rot) = self.listener.lookupTransform(self.fixed_frame, frame_id, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      T = tr.quaternion_matrix(rot)
      T[:3,3] = pos
      try:
        with self.env:
          body.SetTransform(T)
        updated_bodies.append([body.GetName(), frame_id, ''])
      except:
        pass
    # Update the joint values of the OpenRAVE robots
    for robot_name, topic, msg in zip(self.js_robots, self.js_topics, self.js_msgs):
      if msg is None:
        continue
      robot = self.env.GetRobot(robot_name)
      if robot is None:
        continue
      try:
        with self.env:
          robot.SetActiveDOFValues(msg.position)
        updated_robots.append([robot_name, topic])
      except:
        pass
    # Report what we are updating every sec
    report = False
    if event is None:
      report = True
    else:
      if event.last_real is not None:
        self.elapsed_time += (event.current_real - event.last_real).to_sec()
      if self.elapsed_time >= 1.0:
        report = True
        self.elapsed_time = 0.0
    if report:
      state_msg = ''
      if len(updated_robots) > 0:
        state_msg += '\n\n'
        state_msg += tabulate.tabulate(updated_robots, headers=['OpenRAVE Robot', 'ROS Topic'])
      if len(updated_bodies) > 0:
        state_msg += '\n\n'
        state_msg += tabulate.tabulate(updated_bodies, headers=['OpenRAVE KinBody', 'TF frame_id', ''])
      if len(updated_robots+updated_bodies) == 0:
        state_msg += 'Neither robots nor bodies have been updated'
      self.pub.publish(data=state_msg)


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
      if body.GetName() in grabbed_names:
        continue
      Tbody = body.GetTransform()
      body.SetTransform( np.dot(Toffset, Tbody) )

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
