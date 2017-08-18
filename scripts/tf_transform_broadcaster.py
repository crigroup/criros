#!/usr/bin/env python
import os
import yaml
import rospy
import criros
import tf2_ros
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import TransformStamped


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
  rospy.loginfo(
          'Publishing {0} transformation(s) to /tf'.format(len(params_list)))
  tf_broadcaster = tf2_ros.TransformBroadcaster()
  tf_buffer = tf2_ros.Buffer()
  tf_listener = tf2_ros.TransformListener(tf_buffer)
  rate = rospy.Rate(publish_rate)
  tf_msg = TransformStamped()
  while not rospy.is_shutdown():
    for params in params_list:
      trans = np.array(params['translation'])
      rot = np.array(params['rotation'])
      child = params['child']
      parent = params['parent']
      tf_msg.header.stamp = rospy.Time.now()
      if invert:
        T = tr.quaternion_matrix(rot)
        T[:3,3] = trans
        Tinv = criros.spalg.transform_inv(T)
        tf_msg.transform = criros.conversions.to_transform(Tinv)
        tf_msg.header.frame_id = child
        tf_msg.child_frame_id = parent
        tf_broadcaster.sendTransform(tf_msg)
      else:
        tf_msg.transform.translation = criros.conversions.to_vector3(trans)
        tf_msg.transform.rotation = criros.conversions.to_quaternion(rot)
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_broadcaster.sendTransform(tf_msg)
    rate.sleep()
  rospy.loginfo('Shuting down [%s] node' % node_name)
