'''
***************************************************************************
Library of functions written to accompany the algorithms described in
Modern Robotics: Mechanics, Planning, and Control.
***************************************************************************
Author: Mikhail Todes, Huan Weng  
Date: June 2018
***************************************************************************
Language: Python
Also available in: MATLAB, Mathematica
Included libraries: numpy, matplotlib
***************************************************************************
'''

'''
*** IMPORTS ***
'''

import numpy as np
import matplotlib.pyplot as plt

'''
*** BASIC HELPER FUNCTIONS ***
'''

def NearZero(z):
#Takes a scalar.
#Checks if the scalar is small enough to be neglected.
    '''
Example Input:
z = -1e-7
Output:
True
    '''
    return abs(z) < 1e-6
   
def Normalize(V):
#Takes a vector.
#Scales it to a unit vector.
    '''
Example Input: 
V = np.array([1, 2, 3])
Output:
[0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
    '''
    return V / np.linalg.norm(V)

'''
*** CHAPTER 3: RIGID-BODY MOTIONS ***
'''

def RotInv(R):
#Takes a 3x3 rotation matrix.
#Returns the inverse (transpose).
    '''
Example Input: 
R = np.array([[0, 0, 1],
     	      [1, 0, 0],
              [0, 1, 0]])
Output:
[[0, 1, 0], 
 [0, 0, 1],
 [1, 0, 0]]
    '''
    return np.array(R).T

def VecToso3(omg):
#Takes a 3-vector (angular velocity).
#Returns the skew symmetric matrix in so3.
    '''
Example Input: 
omg = np.array([1, 2, 3])
Output:
[[ 0, -3,  2],
 [ 3,  0, -1],
 [-2,  1,  0]]
    '''
    return np.array([[0,      -omg[2],  omg[1]], 
	             [omg[2],       0, -omg[0]], 
	             [-omg[1], omg[0],       0]])

def so3ToVec(so3mat):
#Takes a 3x3 skew-symmetric matrix (an element of so(3)).
#Returns the corresponding vector (angular velocity).
    '''
Example Input: 
so3mat = np.array([[ 0, -3,  2],
          	   [ 3,  0, -1],
                   [-2,  1,  0]])
Output:
[1, 2, 3]
    '''
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def AxisAng3(expc3):
#Takes A 3-vector of exponential coordinates for rotation.
#Returns unit rotation axis omghat and the corresponding rotation angle
#theta.
    '''
Example Input: 
expc3 = np.array([1, 2, 3])
Output:
(array([0.2672612419124244, 0.5345224838248488, 0.8017837257372732]),
 3.7416573867739413) 
    '''
    return (Normalize(expc3), np.linalg.norm(expc3))

def MatrixExp3(so3mat):
#Takes a so(3) representation of exponential coordinates.
#Returns R in SO(3) that is achieved by rotating about omghat by theta from
#an initial orientation R = I.
    '''
Example Input: 
so3mat = np.array([[ 0, -3,  2],
	           [ 3,  0, -1],
                   [-2,  1,  0]])
Output:
[[-0.69492056,  0.71352099,  0.08929286],
 [-0.19200697, -0.30378504,  0.93319235],
 [ 0.69297817,  0.6313497 ,  0.34810748]]
    '''
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

def MatrixLog3(R):
#Takes R (rotation matrix).
#Returns the corresponding so(3) representation of exponential coordinates.
    '''
Example Input: 
R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
Output:
[[          0, -1.20919958,  1.20919958],
 [ 1.20919958,           0, -1.20919958],
 [-1.20919958,  1.20919958,           0]]
    '''
    if NearZero(np.linalg.norm(R - np.eye(3))):
        return np.zeros((3, 3))
    elif NearZero(np.trace(R) + 1):
        if not NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not NearZero(1 + R[1][1]): 
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return VecToso3(np.pi * omg)
    else:
        acosinput = (np.trace(R) - 1) / 2.0
        if acosinput > 1:
            acosinput = 1
        elif acosinput < -1:
            acosinput = -1
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def RpToTrans(R, p):
#Takes rotation matrix R and position p. 
#Returns corresponding homogeneous transformation matrix T in SE(3).
    '''
Example Input: 
R = np.array([[1, 0,  0], 
              [0, 0, -1], 
              [0, 1,  0]])
p = np.array([1, 2, 5])
Output:
[[1, 0,  0, 1],
 [0, 0, -1, 2],
 [0, 1,  0, 5],
 [0, 0,  0, 1]]
    '''
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]    

def TransToRp(T):
#Takes transformation matrix T in SE(3). 
#Returns R: The corresponding rotation matrix,
#        p: The corresponding position vector.
    '''
Example Input: 
T = np.array([[1, 0,  0, 0],
              [0, 0, -1, 0],
              [0, 1,  0, 3],
              [0, 0,  0, 1]])
Output:
(array([[1, 0,  0], 
        [0, 0, -1], 
        [0, 1,  0]]),  
array([0, 0, 3]))
    '''
    R = np.array([[T[0][0], T[0][1], T[0][2]],
                  [T[1][0], T[1][1], T[1][2]],
                  [T[2][0], T[2][1], T[2][2]]])
    return R, np.array([T[0][3], T[1][3], T[2][3]])

def TransInv(T):
#Takes a transformation matrix T. 
#Returns its inverse.
#Uses the structure of transformation matrices to avoid taking a matrix
#inverse, for efficiency.
    '''
Example Input: 
T = np.array([[1, 0,  0, 0],
     	      [0, 0, -1, 0],
              [0, 1,  0, 3],
              [0, 0,  0, 1]])
Output:
[[1,  0, 0,  0],
 [0,  0, 1, -3],
 [0, -1, 0,  0],
 [0,  0, 0,  1]]
    '''
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]
    
def VecTose3(V):
#Takes a 6-vector (representing a spatial velocity). 
#Returns the corresponding 4x4 se(3) matrix.
    '''
Example Input: 
V = np.array([1, 2, 3, 4, 5, 6])
Output:
[[ 0, -3,  2, 4], 
 [ 3,  0, -1, 5], 
 [-2,  1,  0, 6], 
 [ 0,  0,  0, 0]]
    '''
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]

def se3ToVec(se3mat):
#Takes se3mat a 4x4 se(3) matrix.
#Returns the corresponding 6-vector (representing spatial velocity).
    '''
Example Input: 
se3mat = np.array([[ 0, -3,  2, 4], 
                   [ 3,  0, -1, 5], 
                   [-2,  1,  0, 6], 
                   [ 0,  0,  0, 0]])
Output:
[1, 2, 3, 4, 5, 6]
    '''
    return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]],
                 [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]

def Adjoint(T):
#Takes T a transformation matrix SE(3).
#Returns the corresponding 6x6 adjoint representation [AdT].
    '''
Example Input: 
T = np.array([[1, 0,  0, 0], 
              [0, 0, -1, 0], 
              [0, 1,  0, 3], 
              [0, 0,  0, 1]])
Output:
[[1, 0,  0, 0, 0,  0],
 [0, 0, -1, 0, 0,  0],
 [0, 1,  0, 0, 0,  0],
 [0, 0,  3, 1, 0,  0],
 [3, 0,  0, 0, 0, -1],
 [0, 0,  0, 0, 1,  0]]
    '''
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                 np.c_[np.dot(VecToso3(p), R), R]]

def ScrewToAxis(q, s, h):
#Takes q: A point lying on the screw axis, 
#      s: A unit vector in the direction of the screw axis,
#      h: The pitch of the screw axis.
#Returns the corresponding normalized screw axis.
    '''
Example Input: 
q = np.array([3, 0, 0])
s = np.array([0, 0, 1])
h = 2
Output:
[0, 0, 1, 0, -3, 2]
    '''
    return np.r_[s, np.cross(q, s) + np.dot(h, s)]

def AxisAng6(expc6):
#Takes a 6-vector of exponential coordinates for rigid-body motion S*theta.
#Returns S: The corresponding normalized screw axis,
#        theta: The distance traveled along/about S.
    '''
Example Input: 
expc6 = np.array([1, 0, 0, 1, 2, 3])
Output:
([1.0, 0.0, 0.0, 1.0, 2.0, 3.0], 
1.0)
    '''
    theta = np.linalg.norm([expc6[0], expc6[1], expc6[2]])
    if NearZero(theta):
        theta = np.linalg.norm([expc6[3], expc6[4], expc6[5]])
    return (np.array(expc6 / theta), theta)

def MatrixExp6(se3mat):
#Takes a se(3) representation of exponential coordinates.
#Returns a T matrix SE(3) that is achieved by traveling along/about the
#screw axis S for a distance theta from an initial configuration T = I.
    '''
Example Input: 
se3mat = np.array([[0,          0,           0,          0],
          	   [0,          0, -1.57079632, 2.35619449],
                   [0, 1.57079632,           0, 2.35619449],
                   [0,          0,           0,          0]])
Output:
[[1.0, 0.0,  0.0, 0.0],
 [0.0, 0.0, -1.0, 0.0],
 [0.0, 1.0,  0.0, 3.0],
 [  0,   0,    0,   1]]
    '''  
    omgtheta = so3ToVec(np.array(se3mat)[0: 3: 1, 0: 3: 1])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3),
                           [se3mat[0][3], se3mat[1][3], se3mat[2][3]]],
                     [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = np.array(se3mat)[0: 3, 0: 3] / theta
        return np.r_[np.c_[MatrixExp3(np.array(se3mat)[0: 3: 1, 0: 3: 1]),
                           np.dot(np.eye(3) * theta \
                                  + (1 - np.cos(theta)) * omgmat \
                                  + (theta - np.sin(theta)) \
                                    * np.dot(omgmat,omgmat),
                                  [se3mat[0][3],
                                   se3mat[1][3],
                                   se3mat[2][3]]) / theta],
                     [[0, 0, 0, 1]]]

def MatrixLog6(T):
#Takes a transformation matrix T in SE(3).
#Returns the corresponding se(3) representation of exponential coordinates.
    '''
Example Input: 
T = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 3], [0, 0, 0, 1]])
Output:
[[ 0.          0.          0.          0.        ]
 [ 0.          0.         -1.57079633  2.35619449]
 [ 0.          1.57079633  0.          2.35619449]
 [ 0.          0.          0.          0.        ]]
    '''
    R, p = TransToRp(T)
    if NearZero(np.linalg.norm(R - np.eye(3))):
        return np.r_[np.c_[np.zeros((3, 3)),
                           [T[0][3], T[1][3], T[2][3]]],
                     [[0, 0, 0, 0]]]
    else: 
        acosinput = (np.trace(R) - 1) / 2.0
        if acosinput > 1:
            acosinput = 1
        elif acosinput < -1:
            acosinput = -1		
        theta = np.arccos(acosinput)       
        omgmat = MatrixLog3(R) 
        return np.r_[np.c_[omgmat, 
                           np.dot(np.eye(3) - omgmat / 2.0 \
                           + (1.0 / theta - 1.0 / np.tan(theta / 2.0) / 2) \
                             * np.dot(omgmat,omgmat) / theta,[T[0][3], 
                                                              T[1][3], 
                                                              T[2][3]])], 
                     [[0, 0, 0, 0]]]

'''
*** CHAPTER 4: FORWARD KINEMATICS ***
'''

def FKinBody(M, Blist, thetalist):
#Takes M: The home configuration (position and orientation) of the 
#         end-effector,
#      Blist: The joint screw axes in the end-effector frame when the 
#             manipulator is at the home position, in the format of a
#             matrix with axes as the columns,
#      thetalist: A list of joint coordinates.
#Returns T IN SE(3) representing the end-effector frame when the joints are
#at the specified coordinates (i.t.o Body Frame).
    '''
Example Input: 
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
Blist = np.array([[0, 0, -1, 2, 0,   0],
                  [0, 0,  0, 0, 1,   0], 
                  [0, 0,  1, 0, 0, 0.1]]).T
thetalist = np.array([np.pi / 2.0, 3, np.pi])
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
 [  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
 [              0.               0.              -1.       1.68584073],
 [              0.               0.               0.               1.]]
    '''
    T = np.array(M)
    for i in range(len(thetalist)):
        T = np.dot(T,MatrixExp6(VecTose3(np.array(Blist)[:, i] \
                                         * thetalist[i])))              
    return T

def FKinSpace(M, Slist, thetalist):
#Takes M: the home configuration (position and orientation) of the 
#         end-effector,
#      Slist: The joint screw axes in the space frame when the manipulator
#             is at the home position, in the format of a matrix with axes
#             as the columns,
#      thetalist: A list of joint coordinates.
#Returns T in SE(3) representing the end-effector frame when the joints are
#at the specified coordinates (i.t.o Space Frame).
    '''
Example Input: 
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T
thetalist = np.array([np.pi / 2.0, 3, np.pi])
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
 [  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
 [              0.               0.              -1.       1.68584073],
 [              0.               0.               0.               1.]]
    '''
    T = np.array(M)
    for i in range(len(thetalist)-1, -1, -1):
        T = np.dot(MatrixExp6(VecTose3(np.array(Slist)[:, i] \
                                       * thetalist[i])), T)
    return T

'''
*** CHAPTER 5: VELOCITY KINEMATICS AND STATICS***
'''

def JacobianBody(Blist, thetalist):
#Takes Blist: The joint screw axes in the end-effector frame when the
#             manipulator is at the home position, in the format of a
#             matrix with axes as the columns,
#      thetalist: A list of joint coordinates. 
#Returns the corresponding body Jacobian (6xn real numbers).
    '''
Example Input: 
Blist = np.array([[0, 0, 1,   0, 0.2, 0.2], 
                  [1, 0, 0,   2,   0,   3], 
                  [0, 1, 0,   0,   2,   1], 
                  [1, 0, 0, 0.2, 0.3, 0.4]]).T
thetalist = np.array([0.2, 1.1, 0.1, 1.2])
Output:
[[-0.04528405  0.99500417  0.          1. ]
 [ 0.74359313  0.09304865  0.36235775  0. ]
 [-0.66709716  0.03617541 -0.93203909  0. ]
 [ 2.32586047  1.66809     0.56410831  0.2]
 [-1.44321167  2.94561275  1.43306521  0.3]
 [-2.06639565  1.82881722 -1.58868628  0.4]]
    '''
    Jb = np.array(Blist).copy()
    T = np.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = np.dot(T,MatrixExp6(VecTose3(np.array(Blist)[:, i + 1] \
                                         * -thetalist[i + 1])))
        Jb[:, i] = np.dot(Adjoint(T),np.array(Blist)[:, i])  
    return Jb

def JacobianSpace(Slist, thetalist):
#Takes Slist: The joint screw axes in the space frame when the manipulator
#             is at the home position, in the format of a matrix with axes
#             as the columns,
#      thetalist: A list of joint coordinates.
#Returns the corresponding space Jacobian (6xn real numbers).
    '''
Example Input: 
Slist = np.array([[0, 0, 1,   0, 0.2, 0.2], 
                  [1, 0, 0,   2,   0,   3], 
                  [0, 1, 0,   0,   2,   1], 
                  [1, 0, 0, 0.2, 0.3, 0.4]]).T
thetalist = np.array([0.2, 1.1, 0.1, 1.2])
Output:
[[ 0.          0.98006658 -0.09011564  0.95749426]
 [ 0.          0.19866933  0.4445544   0.28487557]
 [ 1.          0.          0.89120736 -0.04528405]
 [ 0.          1.95218638 -2.21635216 -0.51161537]
 [ 0.2         0.43654132 -2.43712573  2.77535713]
 [ 0.2         2.96026613  3.23573065  2.22512443]]
    '''
    Js = np.array(Slist).copy()
    T = np.eye(4)
    for i in range(1, len(thetalist)):
        T = np.dot(T, MatrixExp6(VecTose3(np.array(Slist)[:, i - 1] \
                                * thetalist[i - 1])))
        Js[:,i] = np.dot(Adjoint(T), np.array(Slist)[:, i])  
    return Js
    
'''
*** CHAPTER 6: INVERSE KINEMATICS ***
'''

def IKinBody(Blist, M, T, thetalist0, eomg, ev):
#Takes Blist: The joint screw axes in the end-effector frame when the 
#             manipulator is at the home position, in the format of a
#             matrix with axes as the columns,
#      M: The home configuration of the end-effector,
#      T: The desired end-effector configuration Tsd,
#      thetalist0: An initial guess of joint angles that are close to 
#                  satisfying Tsd,
#      eomg: A small positive tolerance on the end-effector orientation 
#            error. The returned joint angles must give an end-effector 
#            orientation error less than eomg,
#      ev: A small positive tolerance on the end-effector linear position 
#          error. The returned joint angles must give an end-effector 
#          position error less than ev.
#Returns thetalist: Joint angles that achieve T within the specified
#                   tolerances,
#        success: A logical value where TRUE means that the function found 
#                 a solution and FALSE means that it ran through the set 
#                 number of maximum iterations without finding a solution
#                 within the tolerances eomg and ev.
#Uses an iterative Newton-Raphson root-finding method.
#The maximum number of iterations before the algorithm is terminated has 
#been hardcoded in as a variable called maxiterations. It is set to 20 at 
#the start of the function, but can be changed if needed.  
    '''
Example Input: 
Blist = np.array([[0, 0, -1, 2, 0,   0],
                  [0, 0,  0, 0, 1,   0], 
                  [0, 0,  1, 0, 0, 0.1]]).T
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
T = np.array([[0, 1,  0,     -5], 
              [1, 0,  0,      4], 
              [0, 0, -1, 1.6858], 
              [0, 0,  0,      1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001
Output:
thetalist:
array([1.57073819, 2.999667, 3.14153913])
success:
True
    '''
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err)

def IKinSpace(Slist, M, T, thetalist0, eomg, ev):
#Takes Slist: The joint screw axes in the space frame when the manipulator 
#             is at the home position, in the format of a matrix with axes
#             as the columns,
#      M: The home configuration of the end-effector,
#      T: The desired end-effector configuration Tsd,
#      thetalist0: An initial guess of joint angles that are close to 
#                  satisfying Tsd,
#      eomg: A small positive tolerance on the end-effector orientation 
#            error. The returned joint angles must give an end-effector 
#            orientation error less than eomg,
#      ev: A small positive tolerance on the end-effector linear position 
#          error. The returned joint angles must give an end-effector 
#          position error less than ev.
#Returns thetalist: Joint angles that achieve T within the specified 
#                   tolerances,
#        success: A logical value where TRUE means that the function found 
#                 a solution and FALSE means that it ran through the set 
#                 number of maximum iterations without finding a solution
#                 within the tolerances eomg and ev.
#Uses an iterative Newton-Raphson root-finding method.
#The maximum number of iterations before the algorithm is terminated has 
#been hardcoded in as a variable called maxiterations. It is set to 20 at 
#the start of the function, but can be changed if needed.  
    '''
Example Input: 
Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]])
T = np.array([[0, 1,  0,     -5], 
              [1, 0,  0,      4], 
              [0, 0, -1, 1.6858], 
              [0, 0,  0,      1]])
thetalist0 = np.array([1.5, 2.5, 3])
eomg = 0.01
ev = 0.001
Output:
thetalist:
array([1.57073785, 2.99966405, 3.14154125])
success:
True
    '''
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Tsb = FKinSpace(M,Slist, thetalist)
    Vs = np.dot(Adjoint(Tsb), \
                se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T))))
    err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
          or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianSpace(Slist, \
                                                          thetalist)), Vs)
        i = i + 1
        Tsb = FKinSpace(M, Slist, thetalist)
        Vs = np.dot(Adjoint(Tsb), \
                    se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T))))
        err = np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg \
              or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
    return (thetalist, not err)

'''
*** CHAPTER 8: DYNAMICS OF OPEN CHAINS ***
'''

def ad(V):
#Takes 6-vector spatial velocity.
#Returns the corresponding 6x6 matrix [adV].
#Used to calculate the Lie bracket [V1, V2] = [adV1]V2
    '''
Example Input: 
V = np.array([1, 2, 3, 4, 5, 6])
Output:
[[0, -3, 2, 0, 0, 0],
 [3, 0, -1, 0, 0, 0],
 [-2, 1, 0, 0, 0, 0],
 [0, -6, 5, 0, -3, 2],
 [6, 0, -4, 3, 0, -1],
 [-5, 4, 0, -2, 1, 0]]
    '''
    omgmat = VecToso3([V[0], V[1], V[2]])
    return np.r_[np.c_[omgmat, np.zeros((3,3))], 
                 np.c_[VecToso3([V[3], V[4], V[5]]), omgmat]]
 
def InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, \
                    Glist, Slist):
#Takes thetalist: n-vector of joint variables,
#      dthetalist: n-vector of joint rates,
#      ddthetalist: n-vector of joint accelerations,
#      g: Gravity vector g,
#      Ftip: Spatial force applied by the end-effector expressed in frame 
#            {n+1},
#      Mlist: List of link frames {i} relative to {i-1} at the home 
#             position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,.
#Returns taulist: The n-vector of required joint forces/torques.
#This function uses forward-backward Newton-Euler iterations to solve the 
#equation:
#taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) \
#          + g(thetalist) + Jtr(thetalist)Ftip
    '''
Example Input (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
ddthetalist = np.array([2, 1.5, 1])
g = np.array([0, 0, -9.8])
Ftip = np.array([1, 1, 1, 1, 1, 1])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585], 
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ 74.69616155 -33.06766016  -3.23057314]
    '''
    n = len(thetalist)
    Mi = np.eye(4)
    Ai = np.zeros((6, n))
    AdTi = [[None]] * (n + 1)
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g)]
    AdTi[n] = Adjoint(TransInv(Mlist[n]))
    Fi = np.array(Ftip).copy()
    taulist = np.zeros(n)  
    for i in range(n):
        Mi = np.dot(Mi,Mlist[i])
        Ai[:, i] = np.dot(Adjoint(TransInv(Mi)), np.array(Slist)[:, i])
        AdTi[i] = Adjoint(np.dot(MatrixExp6(VecTose3(Ai[:, i] * \
                                            -thetalist[i])), \
                                 TransInv(Mlist[i])))
        Vi[:, i + 1] = np.dot(AdTi[i], Vi[:,i]) + Ai[:, i] * dthetalist[i]
        Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) \
                       + Ai[:, i] * ddthetalist[i] \
                       + np.dot(ad(Vi[:, i + 1]), Ai[:, i]) * dthetalist[i]
    for i in range (n - 1, -1, -1):
        Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) \
             + np.dot(np.array(Glist[i]), Vdi[:, i + 1]) \
             - np.dot(np.array(ad(Vi[:, i + 1])).T, \
                      np.dot(np.array(Glist[i]), Vi[:, i + 1]))
        taulist[i] = np.dot(np.array(Fi).T, Ai[:, i])
    return taulist

def MassMatrix(thetalist, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns.
#Returns M: The numerical inertia matrix M(thetalist) of an n-joint serial 
#           chain at the given configuration thetalist.
#This function calls InverseDynamics n times, each time passing a 
#ddthetalist vector with a single element equal to one and all other inputs
#set to zero. 
#Each call of InverseDynamics generates a single column, and these columns 
#are assembled to create the inertia matrix.
    '''
Example Input (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585], 
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[[  2.25433380e+01  -3.07146754e-01  -7.18426391e-03]
 [ -3.07146754e-01   1.96850717e+00   4.32157368e-01]
 [ -7.18426391e-03   4.32157368e-01   1.91630858e-01]]
    '''
    n = len(thetalist)
    M = np.zeros((n, n))
    for i in range (n):
        ddthetalist = [0] * n
        ddthetalist[i] = 1
        M[:, i] = InverseDynamics(thetalist,[0] * n, ddthetalist, \
                                 [0, 0, 0], [0, 0, 0, 0, 0, 0], Mlist, \
                                 Glist, Slist)
    return M

def VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#      dthetalist: A list of joint rates,
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns.
#Returns c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
#           terms for a given thetalist and dthetalist.
#This function calls InverseDynamics with g = 0, Ftip = 0, and 
#ddthetalist = 0.
    '''
Example Input (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ 0.26453118 -0.05505157 -0.00689132]
    '''
    return InverseDynamics(thetalist, dthetalist, [0] * len(thetalist), \
                           [0, 0, 0], [0, 0, 0, 0, 0, 0], Mlist, Glist, \
                           Slist)

def GravityForces(thetalist, g, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#      g: 3-vector for gravitational acceleration,
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,.
#Returns grav: The joint forces/torques required to overcome gravity at 
#              thetalist
#This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and 
#ddthetalist = 0.
    '''
Example Inputs (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
g = np.array([0, 0, -9.8])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ 28.40331262 -37.64094817  -5.4415892]
    '''
    n = len(thetalist)
    return InverseDynamics(thetalist, [0] * n, [0] * n, g, \
                           [0, 0, 0, 0, 0, 0], Mlist, Glist, Slist)

def EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#      Ftip: Spatial force applied by the end-effector expressed in frame 
#            {n+1},
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,.
#Returns JTFtip: The joint forces and torques required only to create the 
#                end-effector force Ftip.
#This function calls InverseDynamics with g = 0, dthetalist = 0, and 
#ddthetalist = 0.
    '''
Example Input (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
Ftip = np.array([1, 1, 1, 1, 1, 1])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ 1.40954608  1.85771497  1.392409]
    '''
    n = len(thetalist)
    return InverseDynamics(thetalist, [0] * n, [0] * n, [0, 0, 0], Ftip, \
                           Mlist, Glist, Slist)

def ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, \
                    Glist, Slist):
#Takes thetalist: A list of joint variables,
#      dthetalist: A list of joint rates,
#      taulist: An n-vector of joint forces/torques,
#      g: Gravity vector g,
#      Ftip: Spatial force applied by the end-effector expressed in frame
#            {n+1},
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,.
#Returns ddthetalist: The resulting joint accelerations.
#This function computes ddthetalist by solving:
#Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) \
#                              - g(thetalist) - Jtr(thetalist) * Ftip
    '''
Example Input (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
taulist = np.array([0.5, 0.6, 0.7])
g = np.array([0, 0, -9.8])
Ftip = np.array([1, 1, 1, 1, 1, 1])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ -0.97392907  25.58466784 -32.91499212]
    '''
    return np.dot(np.linalg.inv(MassMatrix(thetalist, Mlist, Glist, \
                                           Slist)), \
                  np.array(taulist) \
                  - VelQuadraticForces(thetalist, dthetalist, Mlist, \
                                       Glist, Slist) \
                  - GravityForces(thetalist, g, Mlist, Glist, Slist) \
                  - EndEffectorForces(thetalist, Ftip, Mlist, Glist, \
                                      Slist))
   
def EulerStep(thetalist, dthetalist, ddthetalist, dt):
#Takes thetalist: n-vector of joint variables,
#      dthetalist: n-vector of joint rates,
#      ddthetalist: n-vector of joint accelerations,
#      dt: The timestep delta t.
#Returns thetalistNext: Vector of joint variables after dt from first order
#                       Euler integration,
#        dthetalistNext: Vector of joint rates after dt from first order
#                        Euler integration.
    '''
Example Inputs (3 Link Robot):
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
ddthetalist = np.array([2, 1.5, 1])
dt = 0.1
Output:
thetalistNext:
array([ 0.11,  0.12,  0.13])
dthetalistNext:
array([ 0.3 ,  0.35,  0.4 ])
    '''
    return thetalist + dt * np.array(dthetalist), \
           dthetalist + dt * np.array(ddthetalist)

def InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, \
                              Ftipmat, Mlist, Glist, Slist):
#Takes thetamat: An N x n matrix of robot joint variables,
#      dthetamat: An N x n matrix of robot joint velocities,
#      ddthetamat: An N x n matrix of robot joint accelerations,
#      g: Gravity vector g,
#      Ftipmat: An N x 6 matrix of spatial forces applied by the 
#               end-effector (If there are no tip forces the user should
#               input a zero and a zero matrix will be used),
#      Mlist: List of link frames i relative to i-1 at the home position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,.
#Returns taumat: The N x n matrix of joint forces/torques for the specified
#                trajectory, where each of the N rows is the vector of 
#                joint forces/torques at each time step.
#This function uses InverseDynamics to calculate the joint forces/torques 
#required to move the serial chain along the given trajectory.
    '''
#Example Inputs (3 Link Robot):
from modern_robotics import JointTrajectory
import matplotlib.pyplot as plt
#Create a trajectory to follow using functions from Chapter 9
thetastart =  np.array([0, 0, 0])
thetaend =  np.array([np.pi / 2, np.pi / 2, np.pi / 2])
Tf = 3
N= 1000
method = 5 
traj = JointTrajectory(thetastart, thetaend, Tf, N, method)
thetamat = np.array(traj).copy()
dthetamat = np.zeros((1000,3 ))
ddthetamat = np.zeros((1000, 3))
dt = Tf / (N - 1.0)
for i in range(np.array(traj).shape[0] - 1):
    dthetamat[i + 1, :] = (thetamat[i + 1, :] - thetamat[i, :]) / dt
    ddthetamat[i + 1, :] = (dthetamat[i + 1, :] - dthetamat[i, :]) / dt
#Initialise robot descripstion (Example with 3 links)
g =  np.array([0, 0, -9.8])
Ftipmat = np.ones((N, 6))
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
taumat = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, \
                                   Ftipmat, Mlist, Glist, Slist)
#Output using matplotlib to plot the joint forces/torques
Tau1 = taumat[:, 0]
Tau2 = taumat[:, 1]
Tau3 = taumat[:, 2]
timestamp = np.linspace(0, Tf, N)
plt.plot(timestamp, Tau1, label = "Tau1")
plt.plot(timestamp, Tau2, label = "Tau2")
plt.plot(timestamp, Tau3, label = "Tau3")
plt.ylim (-40, 120)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Torque")
plt.title("Plot of Torque Trajectories")
plt.show()
    '''
    thetamat = np.array(thetamat).T
    dthetamat = np.array(dthetamat).T
    ddthetamat = np.array(ddthetamat).T
    Ftipmat = np.array(Ftipmat).T    
    taumat = np.array(thetamat).copy()
    for i in range(np.array(thetamat).shape[1]):
        taumat[:, i] \
        = InverseDynamics(thetamat[:, i], dthetamat[:, i], \
                          ddthetamat[:, i], g, Ftipmat[:, i], Mlist, \
                          Glist, Slist)
    taumat = np.array(taumat).T
    return taumat

def ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, \
                              Mlist, Glist, Slist, dt, intRes):
#Takes thetalist: n-vector of initial joint variables,
#      dthetalist: n-vector of initial joint rates,
#      taumat: An N x n matrix of joint forces/torques, where each row is 
#              the joint effort at any time step,
#      g: Gravity vector g,
#      Ftipmat: An N x 6 matrix of spatial forces applied by the 
#               end-effector (If there are no tip forces the user should 
#               input a zero and a zero matrix will be used),
#      Mlist: List of link frames {i} relative to {i-1} at the home 
#             position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,
#      dt: The timestep between consecutive joint forces/torques,
#      intRes: Integration resolution is the number of times integration 
#              (Euler) takes places between each time step. Must be an 
#              integer value greater than or equal to 1
#Returns thetamat: The N x n matrix of robot joint angles resulting from 
#                  the specified joint forces/torques,
#        dthetamat: The N x n matrix of robot joint velocities.
#This function simulates the motion of a serial chain given an open-loop
#history of joint forces/torques.
#It calls a numerical integration procedure that uses ForwardDynamics.
    '''
#Example Inputs (3 Link Robot):
import matplotlib.pyplot as plt
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
taumat = np.array([[3.63, -6.58, -5.57], [3.74, -5.55,  -5.5], 
          	   [4.31, -0.68, -5.19], [5.18,  5.63, -4.31],
                   [5.85,  8.17, -2.59], [5.78,  2.79,  -1.7], 
                   [4.99,  -5.3, -1.19], [4.08, -9.41,  0.07],
                   [3.56, -10.1,  0.97], [3.49, -9.41,  1.23]])
#Initialise robot description (Example with 3 links)
g = np.array([0, 0, -9.8])
Ftipmat = np.ones((np.array(taumat).shape[0], 6))
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
dt = 0.1
intRes = 8
thetamat,dthetamat \
= ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, \
                            Mlist, Glist, Slist, dt, intRes)
#Output using matplotlib to plot the joint angle/velocities
theta1 = thetamat[:, 0]
theta2 = thetamat[:, 1]
theta3 = thetamat[:, 2]
dtheta1 = dthetamat[:, 0]
dtheta2 = dthetamat[:, 1]
dtheta3 = dthetamat[:, 2]
N = np.array(taumat).shape[0]
Tf = np.array(taumat).shape[0] * dt
timestamp = np.linspace(0, Tf, N)
plt.plot(timestamp, theta1, label = "Theta1")
plt.plot(timestamp, theta2, label = "Theta2")
plt.plot(timestamp, theta3, label = "Theta3")
plt.plot(timestamp, dtheta1, label = "DTheta1")
plt.plot(timestamp, dtheta2, label = "DTheta2")
plt.plot(timestamp, dtheta3, label = "DTheta3")
plt.ylim (-12, 10)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Joint Angles/Velocities")
plt.title("Plot of Joint Angles and Joint Velocities")
plt.show()
    '''
    taumat = np.array(taumat).T
    Ftipmat = np.array(Ftipmat).T
    thetamat = taumat.copy()
    thetamat[:, 0] = thetalist
    dthetamat = taumat.copy()
    dthetamat[:, 0] = dthetalist
    for i in range(np.array(taumat).shape[1] - 1):        
        for j in range(intRes):
            ddthetalist \
            = ForwardDynamics(thetalist, dthetalist, taumat[:, i], g, \
                              Ftipmat[:, i], Mlist, Glist, Slist)
            thetalist,dthetalist = EulerStep(thetalist, dthetalist, \
                                             ddthetalist, 1.0 * dt / intRes)
        thetamat[:, i + 1] = thetalist
        dthetamat[:, i + 1] = dthetalist
    thetamat = np.array(thetamat).T
    dthetamat = np.array(dthetamat).T
    return thetamat, dthetamat

'''
*** CHAPTER 9: TRAJECTORY GENERATION ***
'''

def CubicTimeScaling(Tf, t):
#Takes Tf: Total time of the motion in seconds from rest to rest,
#      t: The current time t satisfying 0 < t < Tf.
#Returns s: The path parameter s(t) corresponding to a third-order 
#           polynomial motion that begins and ends at zero velocity.
    '''
Example Input: 
Tf = 2
t = 0.6
Output:
0.216
    '''
    return 3 * (1.0 * t / Tf) ** 2 - 2 * (1.0 * t / Tf) ** 3

def QuinticTimeScaling(Tf, t):
#Takes Tf: Total time of the motion in seconds from rest to rest,
#      t: The current time t satisfying 0 < t < Tf.
#Returns s: The path parameter s(t) corresponding to a fifth-order 
#           polynomial motion that begins and ends at zero velocity and 
#           zero acceleration.
    '''
Example Input: 
Tf = 2
t = 0.6
Output:
0.16308
    '''
    return 10 * (1.0 * t / Tf) ** 3 - 15 * (1.0 * t / Tf) ** 4 \
           + 6 * (1.0 * t / Tf) ** 5

def JointTrajectory(thetastart, thetaend, Tf, N, method):
#Takes thetastart: The initial joint variables,
#      thetaend: The final joint variables,
#      Tf: Total time of the motion in seconds from rest to rest,
#      N: The number of points N > 1 (Start and stop) in the discrete
#         representation of the trajectory,
#      method: The time-scaling method, where 3 indicates cubic 
#              (third-order polynomial) time scaling and 5 indicates
#              quintic (fifth-order polynomial) time scaling.
#Returns traj: A trajectory as an N x n matrix, where each row is an 
#              n-vector of joint variables at an instant in time. The first
#              row is thetastart and the Nth row is thetaend . The elapsed 
#              time between each row is Tf/(N - 1).
#The returned trajectory is a straight-line motion in joint space.
    '''
Example Input: 
thetastart = np.array([1, 0, 0, 1, 1, 0.2, 0,1])
thetaend = np.array([1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1])
Tf = 4
N = 6
method = 3
Output:
[[ 1.      0.      0.      1.      1.      0.2     0.      1.    ]
 [ 1.0208  0.052   0.0624  1.0104  1.104   0.3872  0.0936  1.    ]
 [ 1.0704  0.176   0.2112  1.0352  1.352   0.8336  0.3168  1.    ]
 [ 1.1296  0.324   0.3888  1.0648  1.648   1.3664  0.5832  1.    ]
 [ 1.1792  0.448   0.5376  1.0896  1.896   1.8128  0.8064  1.    ]
 [ 1.2     0.5     0.6     1.1     2.      2.      0.9     1.    ]]
    '''
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = np.zeros((len(thetastart), N))
    for i in range(N):
        if method == 3:
            s = CubicTimeScaling(Tf, timegap * i)
        else:
            s = QuinticTimeScaling(Tf, timegap * i)                
        traj[:, i] = s * np.array(thetaend) + (1 - s) * np.array(thetastart)
    traj = np.array(traj).T
    return traj

def ScrewTrajectory(Xstart, Xend, Tf, N, method):
#Takes Xstart: The initial end-effector configuration,
#      Xend: The final end-effector configuration,
#      Tf: Total time of the motion in seconds from rest to rest,
#      N: The number of points N > 1 (Start and stop) in the discrete
#         representation of the trajectory,
#      method: The time-scaling method, where 3 indicates cubic 
#              (third-order polynomial) time scaling and 5 indicates 
#              quintic (fifth-order polynomial) time scaling.
#Returns traj: The discretized trajectory as a list of N matrices in SE(3) 
#              separated in time by Tf/(N-1). The first in the list is 
#              Xstart and the Nth is Xend .
#This function calculates a trajectory corresponding to the screw motion 
#about a space screw axis.
    '''
Example Input: 
Xstart = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
Xend = np.array([[0, 0, 1, 0.1], 
                 [1, 0, 0,   0], 
                 [0, 1, 0, 4.1], 
                 [0, 0, 0,   1]])
Tf = 5
N = 4
method = 3
Output:
[array([[ 1.     0.     0.     1.   ]
        [ 0.     1.     0.     0.   ]
        [ 0.     0.     1.     1.   ]
        [ 0.     0.     0.     1.   ]]),
 array([[ 0.904 -0.25   0.346  0.441]
        [ 0.346  0.904 -0.25   0.529]
        [-0.25   0.346  0.904  1.601]
        [ 0.     0.     0.     1.   ]]),
 array([[ 0.346 -0.25   0.904 -0.117]
        [ 0.904  0.346 -0.25   0.473]
        [-0.25   0.904  0.346  3.274]
        [ 0.     0.     0.     1.   ]]),
 array([[-0.     0.     1.     0.1  ]
        [ 1.    -0.     0.    -0.   ]
        [ 0.     1.    -0.     4.1  ]
        [ 0.     0.     0.     1.   ]])]
    '''
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    for i in range(N):
        if method == 3:
            s = CubicTimeScaling(Tf, timegap * i)
        else:
            s = QuinticTimeScaling(Tf, timegap * i)                
        traj[i] \
        = np.dot(Xstart, MatrixExp6(MatrixLog6(np.dot(TransInv(Xstart), \
                                                     Xend)) * s))
    return traj

def CartesianTrajectory(Xstart, Xend, Tf, N, method):
#Takes Xstart: The initial end-effector configuration,
#      Xend: The final end-effector configuration,
#      Tf: Total time of the motion in seconds from rest to rest,
#      N: The number of points N > 1 (Start and stop) in the discrete 
#         representation of the trajectory,
#      method: The time-scaling method, where 3 indicates cubic 
#              (third-order polynomial) time scaling and 5 indicates 
#              quintic (fifth-order polynomial) time scaling.
#Returns traj: The discretized trajectory as a list of N matrices in SE(3)
#              separated in time by Tf/(N-1). The first in the list is 
#              Xstart and the Nth is Xend.
#This function is similar to ScrewTrajectory, except the origin of the 
#end-effector frame follows a straight line, decoupled from the rotational 
#motion.
    '''
Example Input: 
Xstart = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
Xend = np.array([[0, 0, 1, 0.1], 
                 [1, 0, 0,   0], 
                 [0, 1, 0, 4.1], 
                 [0, 0, 0,   1]])
Tf = 5
N = 4
method = 5
Output:
[array([[ 1.     0.     0.     1.   ]
        [ 0.     1.     0.     0.   ]
        [ 0.     0.     1.     1.   ]
        [ 0.     0.     0.     1.   ]]),
 array([[ 0.937 -0.214  0.277  0.811]
        [ 0.277  0.937 -0.214  0.   ]
        [-0.214  0.277  0.937  1.651]
        [ 0.     0.     0.     1.   ]]),
 array([[ 0.277 -0.214  0.937  0.289]
        [ 0.937  0.277 -0.214  0.   ]
        [-0.214  0.937  0.277  3.449]
        [ 0.     0.     0.     1.   ]]),
 array([[-0.     0.     1.     0.1  ]
        [ 1.    -0.     0.     0.   ]
        [ 0.     1.    -0.     4.1  ]
        [ 0.     0.     0.     1.   ]])]
    '''
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    Rstart, pstart = TransToRp(Xstart)
    Rend, pend = TransToRp(Xend)
    for i in range(N):
        if method == 3:
            s = CubicTimeScaling(Tf, timegap * i)
        else:
            s = QuinticTimeScaling(Tf, timegap * i)                
        traj[i] \
        = np.r_[np.c_[np.dot(Rstart, \
        MatrixExp3(MatrixLog3(np.dot(np.array(Rstart).T,Rend)) * s)), \
                      s * np.array(pend) + (1 - s) * np.array(pstart)], \
                [[0, 0, 0, 1]]]
    return traj

'''
*** CHAPTER 11: ROBOT CONTROL ***
'''

def ComputedTorque(thetalist, dthetalist, eint, g, Mlist, Glist, Slist, \
                   thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd):
#Takes thetalist: n-vector of joint variables,
#      dthetalist: n-vector of joint rates,
#      eint: n-vector of the time-integral of joint errors,
#      g: Gravity vector g,
#      Mlist: List of link frames {i} relative to {i-1} at the home 
#             position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,
#      thetalistd: n-vector of reference joint variables,
#      dthetalistd: n-vector of reference joint velocities,
#      ddthetalistd: n-vector of reference joint accelerations,
#      Kp: The feedback proportional gain (identical for each joint),
#      Ki: The feedback integral gain (identical for each joint),
#      Kd: The feedback derivative gain (identical for each joint).
#Returns taulist: The vector of joint forces/torques computed by the 
#                 feedback linearizing controller at the current instant.
    '''
Example Input: 
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
eint = np.array([0.2, 0.2, 0.2])
g = np.array([0, 0, -9.8])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
thetalistd = np.array([1.0, 1.0, 1.0])
dthetalistd = np.array([2, 1.2, 2])
ddthetalistd = np.array([0.1, 0.1, 0.1])
Kp = 1.3
Ki = 1.2
Kd = 1.1
Output:
[ 133.00525246  -29.94223324   -3.03276856]
    '''
    e = np.subtract(thetalistd, thetalist)
    return np.dot(MassMatrix(thetalist, Mlist, Glist, Slist), \
                  Kp * e + Ki * (np.array(eint) + e) \
                  + Kd * np.subtract(dthetalistd, dthetalist)) \
           + InverseDynamics(thetalist, dthetalist, ddthetalistd, g, \
                             [0, 0, 0, 0, 0, 0], Mlist, Glist, Slist)

def SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, \
                    Slist, thetamatd, dthetamatd, ddthetamatd, gtilde, \
                    Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes):
#Takes thetalist: n-vector of initial joint variables,
#      dthetalist: n-vector of initial joint velocities,
#      g: Actual gravity vector g,
#      Ftipmat: An N x 6 matrix of spatial forces applied by the 
#               end-effector (If there are no tip forces the user should 
#               input a zero and a zero matrix will be used),
#      Mlist: Actual list of link frames i relative to i-1 at the home 
#             position,
#      Glist: Actual spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame, in the format
#             of a matrix with axes as the columns,
#      thetamatd: An Nxn matrix of desired joint variables from the 
#                 reference trajectory,
#      dthetamatd: An Nxn matrix of desired joint velocities,
#      ddthetamatd: An Nxn matrix of desired joint accelerations,
#      gtilde: The gravity vector based on the model of the actual robot 
#              (actual values given above),
#      Mtildelist: The link frame locations based on the model of the 
#                  actual robot (actual values given above),
#      Gtildelist: The link spatial inertias based on the model of the 
#                  actual robot (actual values given above),
#      Kp: The feedback proportional gain (identical for each joint),
#      Ki: The feedback integral gain (identical for each joint),
#      Kd: The feedback derivative gain (identical for each joint),
#      dt: The timestep between points on the reference trajectory,
#      intRes: Integration resolution is the number of times integration 
#              (Euler) takes places between each time step. Must be an 
#              integer value greater than or equal to 1.
#Returns taumat: An Nxn matrix of the controllers commanded joint 
#                forces/torques, where each row of n forces/torques 
#                corresponds to a single time instant,
#        thetamat: An Nxn matrix of actual joint angles.
#The end of this function plots all the actual and desired joint angles 
#using matplotlib and random libraries.
    '''
#Example Input: 
from modern_robotics import JointTrajectory
thetalist = np.array([0.1, 0.1, 0.1])
dthetalist = np.array([0.1, 0.2, 0.3])
#Initialise robot description (Example with 3 links)
g = np.array([0, 0, -9.8])
M01 = np.array([[1, 0, 0,        0], 
                [0, 1, 0,        0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0,        1]])
M12 = np.array([[ 0, 0, 1,    0.28], 
                [ 0, 1, 0, 0.13585],
                [-1, 0, 0,       0],
                [ 0, 0, 0,       1]])
M23 = np.array([[1, 0, 0,       0], 
                [0, 1, 0, -0.1197],
                [0, 0, 1,   0.395], 
                [0, 0, 0,       1]])
M34 = np.array([[1, 0, 0,       0], 
                [0, 1, 0,       0],
                [0, 0, 1, 0.14225], 
                [0, 0, 0,       1]])
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = np.array([G1, G2, G3])
Mlist = np.array([M01, M12, M23, M34])
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
dt = 0.01
#Create a trajectory to follow
thetaend = np.array([np.pi / 2, np.pi, 1.5 * np.pi])
Tf = 1
N = 1.0 * Tf / dt
method = 5
traj = JointTrajectory(thetalist, thetaend, Tf, N, method)
thetamatd = np.array(traj).copy()
dthetamatd = np.zeros((N, 3))
ddthetamatd = np.zeros((N, 3))
dt = Tf / (N - 1.0)
for i in range(np.array(traj).shape[0] - 1):
    dthetamatd[i + 1, :] = (thetamatd[i + 1, :] - thetamatd[i, :]) / dt
    ddthetamatd[i + 1, :] = (dthetamatd[i + 1, :] - dthetamatd[i, :]) / dt
#Possibly wrong robot description (Example with 3 links)
gtilde = np.array([0.8, 0.2, -8.8])
Mhat01 = np.array([[1, 0, 0,   0], 
                   [0, 1, 0,   0], 
                   [0, 0, 1, 0.1], 
                   [0, 0, 0,   1]])
Mhat12 = np.array([[ 0, 0, 1, 0.3], 
                   [ 0, 1, 0, 0.2], 
                   [-1, 0, 0,   0],
                   [ 0, 0, 0,   1]])
Mhat23 = np.array([[1, 0, 0,    0], 
                   [0, 1, 0, -0.2],
                   [0, 0, 1,  0.4], 
                   [0, 0, 0,    1]])
Mhat34 = np.array([[1, 0, 0,   0], 
                   [0, 1, 0,   0],
                   [0, 0, 1, 0.2],
                   [0, 0, 0,   1]])
Ghat1 = np.diag([0.1, 0.1, 0.1, 4, 4, 4])
Ghat2 = np.diag([0.3, 0.3, 0.1, 9, 9, 9])
Ghat3 = np.diag([0.1, 0.1, 0.1, 3, 3, 3])
Gtildelist = np.array([Ghat1, Ghat2, Ghat3])
Mtildelist = np.array([Mhat01, Mhat12, Mhat23, Mhat34])
Ftipmat = np.ones((np.array(traj).shape[0], 6))
Kp = 20
Ki = 10
Kd = 18
intRes = 8
taumat,thetamat \
= SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, Slist, \
                  thetamatd, dthetamatd, ddthetamatd, gtilde, Mtildelist, \
                  Gtildelist, Kp, Ki, Kd, dt, intRes)
    '''
    Ftipmat = np.array(Ftipmat).T
    thetamatd = np.array(thetamatd).T
    dthetamatd = np.array(dthetamatd).T
    ddthetamatd = np.array(ddthetamatd).T
    m,n = np.array(thetamatd).shape
    thetacurrent = np.array(thetalist).copy()
    dthetacurrent = np.array(dthetalist).copy()
    eint = np.zeros((m,1)).reshape(m,)
    taumat = np.zeros(np.array(thetamatd).shape)
    thetamat = np.zeros(np.array(thetamatd).shape)
    for i in range(n):
        taulist \
        = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, \
                        Mtildelist, Gtildelist, Slist, thetamatd[:, i], \
                        dthetamatd[:, i], ddthetamatd[:, i], Kp, Ki, Kd)
        for j in range(intRes):
            ddthetalist \
            = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, \
                             Ftipmat[:, i], Mlist, Glist, Slist)
            thetacurrent, dthetacurrent \
            = EulerStep(thetacurrent, dthetacurrent, ddthetalist, \
                        1.0 * dt / intRes)
        taumat[:, i] = taulist
        thetamat[:, i] = thetacurrent
        eint = np.add(eint, dt * np.subtract(thetamatd[:, i], thetacurrent))
    #Output using matplotlib to plot
    links  = np.array(thetamat).shape[0]
    N = np.array(thetamat).shape[1]
    Tf = N * dt
    timestamp = np.linspace(0, Tf, N)
    #timestampd = np.linspace(0,Tf,(N/intRes))
    for i in range(links):
        col = [np.random.uniform(0, 1), np.random.uniform(0, 1),
               np.random.uniform(0, 1)]
        plt.plot(timestamp, thetamat[i, :], "-", color=col, \
                 label = ("ActualTheta" + str(i + 1)))
        plt.plot(timestamp, thetamatd[i, :], ".", color=col, \
                 label = ("DesiredTheta" + str(i + 1)))
    plt.legend(loc = 'upper left')
    plt.xlabel("Time")
    plt.ylabel("Joint Angles")
    plt.title("Plot of Actual and Desired Joint Angles")
    plt.show()
    taumat = np.array(taumat).T
    thetamat = np.array(thetamat).T    
    return (taumat, thetamat)
