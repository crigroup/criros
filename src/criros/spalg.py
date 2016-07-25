#! /usr/bin/env python
import rospy
import scipy.optimize
import numpy as np
from math import sqrt
import tf.transformations as tr

X_AXIS = np.array([1, 0, 0])
Y_AXIS = np.array([0, 1, 0])
Z_AXIS = np.array([0, 0, 1])

def fit_plane_lstsq(XYZ):
  # Fits a plane to a point cloud, 
  # Where Z = aX + bY + c (1)
  # Rearanging (1): aX + bY -Z +c =0
  # Gives normal (a,b,-1)
  # Normal = (a,b,-1)
  [rows,cols] = XYZ.shape
  G = np.ones((rows,3))
  G[:,0] = XYZ[:,0]  #X
  G[:,1] = XYZ[:,1]  #Y
  Z = XYZ[:,2]
  (a,b,c),resid,rank,s = np.linalg.lstsq(G,Z) 
  normal = (a,b,-1)
  nn = np.linalg.norm(normal)
  normal = normal / nn
  return normal

def fit_plane_optimize(points):
  def f_min(X,p):
    normal = p[0:3]
    d = p[3]
    result = ((normal*X.T).sum(axis=1) + d) / np.linalg.norm(normal) 
    return result
  
  def residuals(params, signal, X):
    return f_min(X, params)
  
  XYZ = np.array(points).T
  p0 = np.array([1,1,1,1])
  sol = scipy.optimize.leastsq(residuals, p0, args=(None, XYZ))[0]
  nn = np.linalg.norm(sol[:3])
  sol /= nn
  rospy.logdebug( 'Solution: ', sol )
  rospy.logdebug( 'Old Error: ', (f_min(XYZ, p0)**2).sum() )
  rospy.logdebug( 'New Error: ', (f_min(XYZ, sol)**2).sum() )
  return sol

def fit_plane_solve(XYZ):
    X = XYZ[:,0]
    Y = XYZ[:,1]
    Z = XYZ[:,2] 
    npts = len(X)
    A = np.array([ [sum(X*X), sum(X*Y), sum(X)],
                   [sum(X*Y), sum(Y*Y), sum(Y)],
                   [sum(X),   sum(Y), npts] ])
    B = np.array([ [sum(X*Z), sum(Y*Z), sum(Z)] ])
    normal = np.linalg.solve(A,B.T)
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return normal.ravel()

def fit_plane_svd(XYZ):
    [rows,cols] = XYZ.shape
    # Set up constraint equations of the form  AB = 0,
    # where B is a column vector of the plane coefficients
    # in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
    p = (np.ones((rows,1)))
    AB = np.hstack([XYZ,p])
    [u, d, v] = np.linalg.svd(AB,0)
    B = v[3,:];                    # Solution is last column of v.
    nn = np.linalg.norm(B[0:3])
    B = B / nn
    return B[0:3]

def force_frame_transform(bTa):
  """
  Calculates the coordinate transformation for force vectors.
  
  The force vectors obey special transformation rules.
  B{Reference:} Handbook of robotics page 39, equation 2.9
  
  @type bTa: array, shape (4,4)
  @param bTa: Homogeneous transformation that represents the position 
  and orientation of frame M{A} relative to frame M{B}
  @rtype: array, shape (6,6)
  @return: The coordinate transformation from M{A} to M{B} for force 
  vectors
  """
  aTb = transform_inv(bTa)
  return motion_frame_transform(aTb).T 

def inertia_matrix_from_vector(i):
  """
  Returns the inertia matrix from its vectorized form.
  
  @type i: array, shape (6,1)
  @param i: The inertia parameters in its vectorized form.
  @rtype: array, shape (3,3)
  @return: The resulting inertia matrix.
  """
  I11 = i[0]
  I12 = i[1]
  I13 = i[2]
  I22 = i[3]
  I23 = i[4]
  I33 = i[5]
  return np.array([ [I11, I12, I13],
                    [I11, I22, I23],
                    [I13, I23, I33]])

def L_matrix(w):
  """
  Returns the 3x6 matrix of angular velocity elements.
  
  @type w: array
  @param w: The angular velocity array
  @rtype: array, shape (3,6)
  @return: The resulting numpy array
  """
  res = np.zeros((3,6))
  res[0,:3] = w.flatten()
  res[1,1:5] = np.insert(w.flatten(),1,0)
  res[2,2:] = np.insert(w.flatten(),1,0)
  return res

def motion_frame_transform(bTa):
  """
  Calculates the coordinate transformation for motion vectors.
  
  The motion vectors obey special transformation rules.
  B{Reference:} Handbook of robotics page 39, equation 2.9
  
  @type bTa: array, shape (4,4)
  @param bTa: Homogeneous transformation that represents the position 
  and orientation of frame M{A} relative to frame M{B}
  @rtype: array, shape (6,6)
  @return: The coordinate transformation from M{A} to M{B} for motion 
  vectors
  """
  bRa = bTa[:3,:3]
  bPa = bTa[:3,3]
  bXa = np.zeros((6,6))
  bXa[:3,:3] = bRa
  bXa[3:,:3] = np.dot(skew(bPa), bRa)
  bXa[3:,3:] = bRa
  return bXa

def perpendicular_vector(v):
  """
  Finds an arbitrary perpendicular vector to B{v}
  """
  v = tr.unit_vector(v)
  if np.allclose(v[:2], np.zeros(2)):
    if np.isclose(v[2], 0.):
      # v is (0, 0, 0)
      raise ValueError('zero vector')
    # v is (0, 0, Z)
    return Y_AXIS
  return np.array([-v[1], v[0], 0])

def rotation_matrix_from_axes(newaxis, oldaxis=Z_AXIS):
  """
  Return the rotation matrix that aligns two vectors.
  """
  oldaxis = tr.unit_vector(oldaxis)
  newaxis = tr.unit_vector(newaxis)
  c = np.dot(oldaxis, newaxis)
  angle = np.arccos(c)
  if np.isclose(c, -1.0) or np.allclose(newaxis, oldaxis):
    v = perpendicular_vector(newaxis)
  else:
    v = np.cross(oldaxis, newaxis)
  return tr.rotation_matrix(angle, v)

def skew(v):
  """
  Returns the 3x3 skew matrix.
  
  The skew matrix is a square matrix M{A} whose transpose is also its 
  negative; that is, it satisfies the condition M{-A = A^T}.
  
  @type v: array
  @param v: The input array
  @rtype: array, shape (3,3)
  @return: The resulting skew matrix
  """
  skv = np.roll(np.roll(np.diag(np.asarray(v).flatten()), 1, 1), -1, 0)
  return (skv - skv.T)

def transformation_estimation_svd(A, B):
  """
  This method implements SVD-based estimation of the transformation 
  aligning the given correspondences.
  
  Estimate a rigid transformation between a source and a target matrices 
  using SVD.
  
  For further information please check: 
    - U{http://dx.doi.org/10.1109/TPAMI.1987.4767965}
    - U{http://nghiaho.com/?page_id=671}
  
  @type A: numpy.array
  @param A: Points expressed in the reference frame A
  @type B: numpy.array
  @param B: Points expressed in the reference frame B
  @rtype: R (3x3), t (3x1)
  @return: (R) rotation matrix and (t) translation vector of the rigid 
  transformation.
  """
  assert A.shape == B.shape
  assert A.shape[1] == 3
  N = A.shape[0]; # total points
  centroid_A = np.mean(A, axis=0)
  centroid_B = np.mean(B, axis=0)
  # centre the points
  AA = A - np.tile(centroid_A, (N, 1))
  BB = B - np.tile(centroid_B, (N, 1))
  # dot is matrix multiplication for array
  H = np.dot(np.transpose(AA), BB)
  U, S, Vt = np.linalg.svd(H)
  R = np.dot(Vt.T, U.T)
  # special reflection case
  if np.linalg.det(R) < 0:
     Vt[2,:] *= -1
     R = np.dot(Vt.T, U.T)
  t = -np.dot(R, centroid_A.T) + centroid_B.T
  return R, t

def transformation_between_planes(newplane, oldplane):
  """
  Returns the transformation matrix that aligns two planes.
  """
  newaxis = np.array(newplane[:3])
  Rnew = rotation_matrix_from_axes(newaxis, Z_AXIS)[:3,:3]
  newpoint = np.dot(Rnew, np.array([0,0,newplane[3]]))
  oldaxis = np.array(oldplane[:3])
  Rold = rotation_matrix_from_axes(oldaxis, Z_AXIS)[:3,:3]
  oldpoint = np.dot(Rold, np.array([0,0,oldplane[3]]))
  T = rotation_matrix_from_axes(newaxis, oldaxis)
  T[:3,3] = -newpoint + np.dot(T[:3,:3], oldpoint)
  return T

def transform_inv(T):
  """
  Calculates the inverse of the input homogeneous transformation.
  
  This method is more efficient than using C{numpy.linalg.inv}, given 
  the special properties of the homogeneous transformations.
  
  @type T: array, shape (4,4)
  @param T: The input homogeneous transformation
  @rtype: array, shape (4,4)
  @return: The inverse of the input homogeneous transformation
  """
  R = T[:3,:3].T
  p = T[:3,3]
  T_inv = np.identity(4)
  T_inv[:3,:3] = R
  T_inv[:3,3] = np.dot(-R, p)
  return T_inv
