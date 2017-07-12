#! /usr/bin/env python
import unittest
import criros.rviz


class TestrvizModule(unittest.TestCase):
  def test_create_interactive_6dof(self):
    imarker = criros.rviz.create_interactive_6dof('name')
  
  def test_create_interactive_mesh(self):
    imarker = criros.rviz.create_interactive_mesh('name', 'resource')
  
  def test_create_mesh_marker(self):
    marker = criros.rviz.create_mesh_marker(1, 'name', 'resource')
  
  def test_create_points_marker(self):
    points = [[1,2,3] for _ in range(10)]
    marker = criros.rviz.create_points_marker(1, points)
  
  def test_create_text_marker(self):
    marker = criros.rviz.create_text_marker(1, 'text')
  
  def test_get_safe_stamp(self):
    stamp = criros.rviz.get_safe_stamp()
