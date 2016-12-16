#!/usr/bin/env python
import os
import tf
import rospy
import criros
import numpy as np
import tf.transformations as tr


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  logger=rospy
  logger.loginfo('Starting [%s] node' % node_name)
  # Read publish rate
  publish_rate = criros.read_parameter('~publish_rate', 60.0)
  invert = criros.read_parameter('~invert', False)
  # Read all the other parameters
  try:
    params = rospy.get_param('~')
  except KeyError:
    logger.logwarn('No parameters found on parameter server')
    exit(0)
  expected_keys = ['parent', 'child', 'rotation', 'translation']
  params_list = []
  for key, data in params.items():
    if not criros.utils.has_keys(data, expected_keys):
      continue
    params_list.append(data)
  if len(params_list) == 0:
    logger.logwarn('No transformations found on parameter server')
    exit(0)
  # Publish tf data
  rospy.loginfo('Publishing {0} transformation(s) to /tf'.format(len(params_list)))
  broadcaster = tf.TransformBroadcaster()
  listener = tf.TransformListener()
  rate = rospy.Rate(publish_rate)
  while not rospy.is_shutdown():
    for params in params_list:
      trans = np.array(params['translation'])
      rot = np.array(params['rotation'])
      child = params['child']
      # If using gazebo, the parent of base_link will be world.
      # Therefore, we need to change the child to world
      if 'base_link' in child:
        newchild = child.replace('base_link', 'world')
        if listener.frameExists(newchild):
          child = newchild
      parent = params['parent']
      if not invert:
        broadcaster.sendTransform(trans, rot, rospy.Time.now(), child, parent)
      else:
        T = tr.quaternion_matrix(rot)
        T[:3,3] = trans
        Tinv = criros.spalg.transform_inv(T)
        trans_inv = Tinv[:3,3]
        rot_inv = tr.quaternion_from_matrix(Tinv)
        broadcaster.sendTransform(trans_inv, rot_inv, rospy.Time.now(), parent, child)
    rate.sleep()
  rospy.loginfo('Shuting down [%s] node' % node_name)
