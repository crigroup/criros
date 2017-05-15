#! /usr/bin/env python
import cv2
import numpy as np


def draw_circles(image, circles, color=(0,255,0), thickness=1, center_color=(0,0,255), center_thickness=2):
  if circles is None:
    return
  for circle in np.uint16(np.around(circles))[0,:]:
    center = tuple(circle[:2])
    radius = circle[2]
    cv2.circle(image, center, radius, color=color, thickness=thickness)
    cv2.circle(image, center, center_thickness, color=center_color, thickness=cv2.FILLED)

def draw_roi(image, roi, color=(255,0,0), thickness=2):
  roi = tuple(map(tuple, roi))
  cv2.rectangle(image, *roi, color=color, thickness=thickness)

def extract_roi(image, roi):
  return image[roi[0][1]:roi[1][1], roi[0][0]:roi[1][0]]
