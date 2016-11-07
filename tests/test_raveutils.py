#! /usr/bin/env python
import yaml
import time
import criros
import unittest
import numpy as np
import openravepy as orpy


class TestraveutilsModule(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    np.set_printoptions(precision=6, suppress=True)
    cls.env = orpy.Environment()
    if not cls.env.Load('data/lab1.env.xml'):
      raise Exception('Could not load scene: data/lab1.env.xml')
    print('') # dummy line
  
  @classmethod
  def tearDownClass(cls):
    cls.env.Reset()
    cls.env.Destroy()
  
  def test_compute_bounding_box_corners(self):
    env = self.env
    body = env.GetKinBody('mug1')
    corners = criros.raveutils.compute_bounding_box_corners(body, Tbody=None, scale=1.0)
    expected_corners = [[-0.109647, -0.293446,  0.755   ],
                        [-0.109647, -0.293446,  0.86422 ],
                        [-0.109647, -0.187754,  0.755   ],
                        [-0.109647, -0.187754,  0.86422 ],
                        [ 0.026916, -0.293446,  0.755   ],
                        [ 0.026916, -0.293446,  0.86422 ],
                        [ 0.026916, -0.187754,  0.755   ],
                        [ 0.026916, -0.187754,  0.86422 ]]
    np.testing.assert_allclose(expected_corners, corners, atol=1e-5)
  
  def test_environment_from_dict(self):
    env = self.env
    # Test loading a environment
    yaml_text = """
    openrave:
      world: data/lab1.env.xml
      viewer:
        name: default
        camera:
          translation: [3, 0, 2.5]
          rotation: [ 0.595,  0.595, -0.382, -0.382 ]
    """
    env.Reset()
    config = yaml.load(yaml_text)['openrave']
    configured_env = criros.raveutils.environment_from_dict(config, env=env)
    self.assertIsNotNone(configured_env)
    # Check it loaded all the bodies
    expected_bodies = ['BarrettWAM',
                       'floorwalls',
                       'pole',
                       'pole2',
                       'pole3',
                       'wall1',
                       'segway',
                       'mug1',
                       'mug2',
                       'mug3',
                       'mug4',
                       'mug5',
                       'mug6',
                       'dishwasher_table',
                       'table']
    found_bodies = [body.GetName() for body in env.GetBodies()]
    self.assertEqual(set(expected_bodies), set(found_bodies))
    # Check the viewer camera transformation is correct
    viewer = env.GetViewer()
    # Wait for the viewer to start
    while np.allclose(viewer.GetCameraTransform()[:3,3], np.zeros(3), atol=1e-5):
      time.sleep(0.01)
    Texpected = criros.conversions.from_dict(config['viewer']['camera'])
    Tcamviewer = viewer.GetCameraTransform()
    np.testing.assert_allclose(Texpected, Tcamviewer, atol=1e-5)
  
  def test_get_enabled_bodies(self):
    env = self.env
    expected = set([body.GetName() for body in env.GetBodies()])
    enabled_bodies = criros.raveutils.get_enabled_bodies(env)
    self.assertEqual(expected, enabled_bodies)
  
  def test_get_robot_iktypes(self):
    env = self.env
    robot = env.GetRobot('BarrettWAM')
    iktypes = criros.raveutils.get_robot_iktypes(robot)
    self.assertEqual({'arm':[criros.raveutils.iktype6D]}, iktypes)
  
  def test_move_origin_to_body(self):
    env = self.env
    # At the beginning, the origin is at the floorwalls body
    floorwalls = env.GetKinBody('floorwalls')
    np.testing.assert_allclose(floorwalls.GetTransform(), np.eye(4))
    # Move origin to the robot
    robot = env.GetRobot('BarrettWAM')
    Tinit_robot = robot.GetTransform()
    criros.raveutils.move_origin_to_body(robot)
    np.testing.assert_allclose(robot.GetTransform(), np.eye(4))
    # The floorwalls now must have the inv(Tinit_robot)
    np.testing.assert_allclose(floorwalls.GetTransform(), criros.spalg.transform_inv(Tinit_robot))
  
  def test_random_joint_positions(self):
    env = self.env
    robot = env.GetRobot('BarrettWAM')
    lower, upper = robot.GetActiveDOFLimits()
    # Test 1000 random values
    for _ in range(1000):
      random_values = criros.raveutils.random_joint_positions(robot)
      self.assertTrue(np.all(random_values > lower))
      self.assertTrue(np.all(random_values < upper))
  
  def test_remove_bodies(self):
    env = self.env
    # Remove all the mugs
    remove = [body.GetName() for body in env.GetBodies() if ('mug' in body.GetName()) and (body.GetName() != 'mug1') ]
    criros.raveutils.remove_bodies(env, remove=remove)
    expected_bodies = ['BarrettWAM',
                       'floorwalls',
                       'mug1',
                       'pole',
                       'pole2',
                       'pole3',
                       'wall1',
                       'segway',
                       'dishwasher_table',
                       'table']
    found_bodies = [body.GetName() for body in env.GetBodies()]
    self.assertEqual(set(expected_bodies), set(found_bodies))
    # Keep only the robot and mug1
    criros.raveutils.remove_bodies(env, keep=['BarrettWAM', 'mug1'])
    found_bodies = [body.GetName() for body in env.GetBodies()]
    self.assertEqual(set(['BarrettWAM', 'mug1']), set(found_bodies))
  
  def test_set_body_transparency(self):
    # Helper function
    def check_transparency(body, transparency):
      for link in body.GetLinks():
        for geom in link.GetGeometries():
          self.assertEqual(transparency, geom.GetTransparency())
    env = self.env
    robot = env.GetRobot('BarrettWAM')
    criros.raveutils.set_body_transparency(robot, transparency=0.5)
    check_transparency(robot, 0.5)
    # Check that we clip the transparency to be between 0 and 1
    criros.raveutils.set_body_transparency(robot, transparency=-2)
    check_transparency(robot, 0.0)
    criros.raveutils.set_body_transparency(robot, transparency=2)
    check_transparency(robot, 1.0)
  
  def test_trimesh_from_point_cloud(self):
    env = self.env
    body = env.GetKinBody('mug1')
    corners = criros.raveutils.compute_bounding_box_corners(body, Tbody=None, scale=1.0)
    trimesh = criros.raveutils.trimesh_from_point_cloud(corners)
    testbody = orpy.RaveCreateKinBody(env, '')
    testbody.SetName('testbody')
    testbody.InitFromTrimesh(trimesh, draw=True)
    env.Add(testbody, True)
    # Compare AABBs
    body_aabb = body.ComputeAABB()
    testbody_aabb = testbody.ComputeAABB()
    np.testing.assert_allclose(body_aabb.pos(), testbody_aabb.pos())
    np.testing.assert_allclose(body_aabb.extents(), testbody_aabb.extents())
