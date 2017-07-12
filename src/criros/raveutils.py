#! /usr/bin/env python
import copy
import criros
import itertools
import numpy as np
import scipy.spatial
import sklearn.cluster
import openravepy as orpy
# Transformations
import tf.transformations as tr
# Logger
from criros.utils import TextColors

logger = TextColors()

# Supported IK types
iktype5D = orpy.IkParameterization.Type.TranslationDirection5D
iktype6D = orpy.IkParameterization.Type.Transform6D
SUPPORTED_IK_TYPES = [iktype5D, iktype6D]


class Hole(object):
  def __init__(self, position, direction, depth):
    self.position = np.array(position)
    self.direction = tr.unit_vector(direction)
    self.depth = abs(depth)
    
  def __repr__(self):
    printoptions = np.get_printoptions()
    np.set_printoptions(precision=4, suppress=True)
    text = '<Hole(pos: {0} dir: {1} depth: {2})>'.format(self.position, self.direction, self.depth)
    np.set_printoptions(**printoptions)
    return text
  
  def __str__(self):
    return self.__repr__()
  
  def get_ray(self):
    return orpy.Ray(self.position, self.direction)
  
  def transform(self, T):
    ray = self.get_ray()
    Thole = criros.conversions.from_ray(ray)
    Tnew = np.dot(T, Thole)
    newray = criros.conversions.to_ray(Tnew)
    self.position = newray.pos()
    self.direction = newray.dir()


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

def enable_body(body, enable):
  """
  Enables all the links of a body.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @type  enable: bool
  @param enable: If true, will enable all the links.
  """
  env = body.GetEnv()
  with env:
    for link in body.GetLinks():
      link.Enable(enable)

def environment_from_dict(config, env=None, logger=TextColors()):
  """
  Loads and configures and OpenRAVE environment from a configuration dictionary.
  This approach allows to encapsulate additional information that would be tedious
  to include if we only used the OpenRAVE XML specification.
  @type  config: dict
  @param config: The configuration dictionary
  @rtype: orpy.Environment
  @return: The OpenRAVE environment loaded
  """
  if not isinstance(config, dict):
    logger.logwarn('config is not a dict')
    return None
  # Check required fields are in the config dict
  required_fields = ['world']
  if not criros.utils.has_keys(config, required_fields):
    logger.logwarn( 'config dict does not have the required fields: {0}'.format(required_fields) )
    return None
  if env is None:
    env = orpy.Environment()
  if not env.Load(config['world']):
    env = None
    return None
  # OPTIONAL parameters
  # Viewer parameters
  if config.has_key('viewer'):
    viewer_name = config['viewer']['name']
    if viewer_name == 'default':
      env.SetDefaultViewer()
    else:
      env.SetViewer(viewer_name)
    # The camera where we look the viewer from
    if config['viewer'].has_key('camera'):
      transform_dict = config['viewer']['camera']
      camera_fields = ['rotation','translation']
      if not criros.utils.has_keys(transform_dict, camera_fields):
        logger.logwarn('camera dict does not have the required fields: {0}'.format(camera_fields))
      elif env.GetViewer() is not None:
        Tcam = criros.conversions.from_dict(transform_dict)
        env.GetViewer().SetCamera(Tcam)
  # Return configured environment
  return env

def destroy_env(env):
  """
  Dummy function that destroys properly an OpenRAVE environment.
  @note: Useful when working with C{IPython} + QtCoin viewer.
  @type  env: orpy.Environment
  @param env: The OpenRAVE environment
  """
  env.Reset()
  env.Destroy()

def find_body_holes(body, radius, absolute=True):
  import trimesh
  mesh_holes = dict()
  body_holes = dict()
  Tbody = body.GetTransform()
  for link in body.GetLinks():
    link_holes = []
    Tlink = link.GetTransform()
    for geometry in link.GetGeometries():
      if geometry.GetType() == orpy.GeometryType.Trimesh:
        filename = geometry.GetRenderFilename()
        scale = geometry.GetRenderScale()
        pose = geometry.GetTransformPose()
        if filename not in mesh_holes:
          mesh = trimesh.load(filename)
          mesh_holes[filename] = find_mesh_holes(mesh.vertices, mesh.faces, radius, scale)
        for h in mesh_holes[filename]:
          hole = copy.deepcopy(h)
          Tmesh = np.dot(Tlink, orpy.matrixFromPose(pose))
          if absolute:
            hole.transform(np.dot(Tbody,Tmesh))
          else:
            hole.transform(Tmesh)
          link_holes.append(hole)
    if len(link_holes) > 0:
      body_holes[str(link.GetName())] = link_holes
  return body_holes

def find_mesh_holes(vert, faces, radius, scale=1., fitplane_eps=1e-8, 
                                                  fitplane_attempts=10):
  vertices = np.array(vert)*scale
  # Circles have lots of vertices. Use clustering to locate them
  eps = radius + 1e-3
  db = sklearn.cluster.DBSCAN(eps=eps, min_samples=10).fit(vertices)
  unique_labels = set(db.labels_)
  circles_info = []
  for k in unique_labels:
    if k == -1:   # Unknown cluster
      continue
    points = vertices[db.labels_==k]
    for _ in range(fitplane_attempts):
      seed = np.zeros(4)
      seed[:3] = tr.unit_vector(tr.random_vector(3))
      res = criros.spalg.fit_plane_optimize(points, seed=seed)
      equation=res[0]
      fit_error = res[2]
      if fit_error < fitplane_eps:
        break
    data = dict()
    data['center'] = np.mean(points, axis=0)
    data['plane'] = criros.spalg.Plane(equation=res[0])
    circles_info.append(data)
    if fit_error > fitplane_eps:
      # Report circles that weren't fitted properly
      print 'Circle planefit error above threshold: {0}'.format(fit_error)
  # One hole is composed by two circles, pair them
  holes = []
  num_circles = len(circles_info)
  found = set()
  for i in range(num_circles):
    if i in found:
      continue  # Skip already paired circles
    for j in range(1, num_circles):
      if (j in found) or (i == j):
        continue  # Skip already paired circles
      plane_i = circles_info[i]['plane']
      plane_j = circles_info[j]['plane']
      ni = plane_i.normal
      nj = plane_j.normal
      parallel = np.isclose(abs(np.dot(ni,nj)), 1.)
      center_i = circles_info[i]['center']
      center_j = circles_info[j]['center']
      pi = center_i
      pj = plane_i.project(center_j)
      if parallel and np.allclose(pi, pj):
        found.add(i)
        found.add(j)
        position = center_j
        diff = center_i - center_j
        direction = tr.unit_vector(diff)
        depth = np.linalg.norm(diff)
        holes.append(Hole(position, direction, depth))
  return holes

def generate_convex_decomposition_model(robot, padding):
  cdmodel = orpy.databases.convexdecomposition.ConvexDecompositionModel(robot, padding=padding)
  cdmodel.generate(padding=padding, minTriangleConvexHullThresh=12000, skinWidth=0, decompositionDepth=8, maxHullVertices=256, concavityThresholdPercent=10, mergeThresholdPercent=30, volumeSplitThresholdPercent=15)
  cdmodel.save()
  if cdmodel.load():
    return cdmodel
  else:
    return None

def get_arm_length_estimate(robot):
  """
  The best estimate of arm length is to sum up the distances of the anchors of all the points in between the chain
  """
  manip = robot.GetActiveManipulator()
  armjoints = [j for j in robot.GetDependencyOrderedJoints() if j.GetJointIndex() in manip.GetArmIndices()]
  baseanchor = armjoints[0].GetAnchor()
  eetrans = manip.GetEndEffectorTransform()[0:3,3]
  armlength = 0
  for j in armjoints[::-1]:
    armlength += np.sqrt(np.sum((eetrans-j.GetAnchor())**2))
    eetrans = j.GetAnchor()
  return armlength

def get_enabled_bodies(env):
  """
  Returns a C{set} with the names of the bodies enabled in the given environment
  @type  env: orpy.Environment
  @param env: The OpenRAVE environment
  @rtype: set
  @return: The names of the enabled bodies
  """
  enabled_bodies = []
  with env:
    for body in env.GetBodies():
      if body.IsEnabled():
        enabled_bodies.append(body.GetName())
  return set(enabled_bodies)

def get_robot_iktypes(robot):
  """
  Returns a dict with the manipulator:[iktypes] pairs of available iksolvers . 
  @type  refbody: orpy.Robot
  @param refbody: The OpenRAVE robot
  @rtype: orpy.Environment
  @return: The dict with the manipname:[iktypes] pairs.
  """
  robot_iktypes = dict()
  for manip in robot.GetManipulators():
    iktypes = []
    for iktype in SUPPORTED_IK_TYPES:
      ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(iktype=iktype, manip=manip)
      if ikmodel.load():
        iktypes.append(iktype)
    if iktypes:
      robot_iktypes[manip.GetName()] = list(iktypes)
  return robot_iktypes

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


def move_out_of_collision(env, body, max_displacement=0.005):
  """
  Moves an OpenRAVE body out of collision in the opposite direction to the penetration direction.
  @type  env: orpy.Environment
  @param env: The OpenRAVE environment.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body.
  @type  max_displacement: float
  @param max_displacement: The maximum displacement we can apply to the body.
  """
  if not env.CheckCollision(body):
    # Not in collision
    return True
  # Need to use pqp collision checker
  previous_cc = env.GetCollisionChecker()
  checker = orpy.RaveCreateCollisionChecker(env, 'pqp')
  checker.SetCollisionOptions(orpy.CollisionOptions.Distance|orpy.CollisionOptions.Contacts)
  env.SetCollisionChecker(checker)
  # Collision report
  report = orpy.CollisionReport()
  env.CheckCollision(body, report)
  # Restore previous collision checker
  env.SetCollisionChecker(previous_cc)
  # Get the direction we should push the object
  positions = []
  normals = []
  occurrences = []
  for c in report.contacts:
    positions.append(c.pos)
    if len(normals) == 0:
      normals.append(c.norm)
      occurrences.append(1)
      continue
    found = False
    for i,normal in enumerate(normals):
      if np.allclose(c.norm, normal):
        occurrences[i] += 1
        found = True
        break
    if not found:
      normals.append(c.norm)
      occurrences.append(1)
  push_direction = tr.unit_vector(normals[np.argmax(occurrences)])
  # Get the distance we should push the object
  Tbody = body.GetTransform()
  Tnew = np.array(Tbody)
  push_distance = 0
  while env.CheckCollision(body):
    push_distance += 0.001
    Tnew[:3,3] = Tbody[:3,3] + push_distance*push_direction
    body.SetTransform(Tnew)
    if push_distance > max_displacement:
      print 'push_distance: {0}'.format(push_distance)
      body.SetTransform(Tbody)
      return False
  return True

def random_joint_positions(robot):
  """
  Generates random joint positions within joint limits for the given robot.
  @type  robot: orpy.Robot
  @param robot: The OpenRAVE robot
  @rtype: np.array
  @return: 
  """
  # Get the limits of the active DOFs
  lower, upper = robot.GetActiveDOFLimits()
  positions = lower + np.random.rand(len(lower))*(upper-lower)
  return positions

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

def remove_body_padding(body):
  """
  Restores the collision meshes of the body. The original collision 
  meshes are store as C{UserData} by the C{set_body_padding} function.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @rtype: bool
  @return: True if succeeded, False otherwise
  """
  if not body.IsRobot():
    raise Exception('Not implemented yet for bodies')
  robot = body
  original_collision_meshes = robot.GetUserData()
  if original_collision_meshes is None:
    logger.logerr('Robot user data is empty: {0}'.format(robot.GetName()))
    return False
  for name,meshes in original_collision_meshes.items():
    link = robot.GetLink(name)
    for geom,mesh in itertools.izip(link.GetGeometries(), meshes):
      if mesh is not None:
        geom.SetCollisionMesh(mesh)
  return True

def set_body_padding(body, padding, generate=False, links=[]):
  """
  Sets the padding for the specified links. If C{links} is empty, 
  the padding will be done for ALL the links.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @type  padding: float
  @param padding: The padding value.
  @type  generate: bool
  @param generate: If set, the ConvexDecompositionModel will be 
  generated if it doesn't exist already.
  @type  links: list
  @param links: The list of links to be padded. If it is empty, 
  the padding will be done for ALL the links.
  @rtype: bool
  @return: True if succeeded, False otherwise
  """
  if not body.IsRobot():
    raise Exception('Not implemented yet for bodies')
  robot = body
  cdmodel = orpy.databases.convexdecomposition.ConvexDecompositionModel(robot, padding=padding)
  if not cdmodel.load():
    if generate:
      cmodel = generate_convex_decomposition_model(robot, padding)
      if cdmodel is None:
        logger.logerr('Failed to generate ConvexDecompositionModel: {0}'.format(robot.GetName()))
        return False
    else:
      logger.logerr('ConvexDecompositionModel database for robot {0} with padding {1:.3f} not found'.format(robot.GetName(), padding))
      return False
  if len(links) == 0:
    # Do it for all the links
    links = [l.GetName() for l in robot.GetLinks()]
  original_collision_meshes = robot.GetUserData()
  if original_collision_meshes is None:
    original_collision_meshes = dict()
  env = robot.GetEnv()
  with env:
    for link, linkcd in itertools.izip(robot.GetLinks(), cdmodel.linkgeometry):
      if link.GetName() not in links:
        continue
      make_a_copy = link.GetName() not in original_collision_meshes
      if make_a_copy:
        original_collision_meshes[link.GetName()] = [None] * len(link.GetGeometries())
      for ig,hulls in linkcd:
        geom = link.GetGeometries()[ig]
        if geom.IsModifiable():
          if make_a_copy:
            # Keep a copy of the original collision meshes
            original_collision_meshes[link.GetName()][ig] = geom.GetCollisionMesh()
          # Set the padded mesh
          geom.SetCollisionMesh(cdmodel.GenerateTrimeshFromHulls(hulls))
  robot.SetUserData(original_collision_meshes)
  return True

def set_body_transparency(body, transparency=0.0, links=None):
  """
  Sets the transparency value of a body recursively.
  @type  body: orpy.KinBody
  @param body: The OpenRAVE body
  @type  transparency: float
  @param transparency: The transparency value. If it's out of range [0.0, 1.0], it'll be clipped.
  @type  links: list
  @param links: Links to be changed. By default all the links are changed
  """
  transparency = np.clip(transparency, 0.0, 1.0)
  env = body.GetEnv()
  with env:
    for link in body.GetLinks():
      if type(links) == list:
        if link.GetName() not in links:
          continue
      for geom in link.GetGeometries():
        geom.SetTransparency(transparency)

def trimesh_from_point_cloud(cloud):
  """
  Converts a PCL point cloud into a OpenRAVE trimesh
  @type  cloud: pcl.Cloud
  @param cloud: The PCL cloud
  @rtype: orpy.Trimesh
  @return: The OpenRAVE trimesh
  """
  points = np.asarray(cloud)
  hull = scipy.spatial.ConvexHull(points)
  hull = scipy.spatial.ConvexHull(points[hull.vertices])
  criros.spalg.counterclockwise_hull(hull)
  return orpy.TriMesh(hull.points, hull.simplices)
