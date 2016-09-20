'''
**********************************************************************************************
Library of functions written to accompany the algorithms described in Introduction to Robotics: Mechanics, Planning, and Control.
**********************************************************************************************
**********************************************************************************************
Author: Mikhail Todes
Email: mikhail@u.northwestern.edu
Github: https://github.com/MikhailTodes
Date: November 2015
**********************************************************************************************
Language: Python
Also available in: MATLAB, Mathematica
Included libraries: numpy, math, matplotlib, random
**********************************************************************************************
'''

'''
**********************************************************************************************
**************************************  IMPORTS  *********************************************
**********************************************************************************************
'''
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt
import random


'''
**********************************************************************************************
*******************************  BASIC HELPER FUNCTIONS  *************************************
**********************************************************************************************
'''
def matmult(*x):#Returns the dot product of the inputs
    '''
Example Input:
([1,2,3], [3,2,1])
Output:
10
    '''
    try:
        return reduce(np.dot, x)
    except:
        print ("Input vectors are not aligned")


def Magnitude(V):#Takes in a vector and returns its length
    '''
Example Input: 
V = [1,2,3]
Output:
3.74165738677
    '''
    try:
        length = 0
        for i in range(len(V)):
            length += (V[i]**2)
        return ((length)**0.5)
    except:
        print ("Input is not a vector")


def Normalize(V):#Takes in a vector and scales it to a unit vector
    '''
Example Input: 
V = [1,2,3]
Output:
[0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
    '''
    try:
        length = Magnitude(V)
        if (length == 0.):
            for i in range(len(V)):
                V[i] = 0.0
            return V
        for i in range(len(V)):
            V[i] = V[i]/length
        return V
    except:
        print ("Input is not a vector")


def det(R):#Takes a square matrix and returns its determinant
    '''
Example Input: 
R = [[0, 0,1],
    [1, 0, 0],
    [0, 1, 0]]
Output:
1.0
    '''
    try:
        return np.linalg.det(R)
    except:
        print("Matrix is not square")



'''
**********************************************************************************************
****************************  CHAPTER 3: RIGID-BODY MOTIONS  *********************************
**********************************************************************************************
'''
def RotInv(R):#Takes a 3x3 rotation matrix and returns the transpose (inverse)
    '''
Example Input: 
R = [[0, 0,1],
    [1, 0, 0],
    [0, 1, 0]]
Output:
[[0, 1, 0], 
[0, 0, 1],
[1, 0, 0]]
    '''
    try:
        #Test for determinant = 1 +- 0.05 error
        if (det(R)*1.0>=0.95 and det(R)*1.0<=1.05):
            invR = [[R[0][0],R[1][0],R[2][0]],[R[0][1],R[1][1],R[2][1]],[R[0][2],R[1][2],R[2][2]]]
        else:
            print ("Determinant of the input matrix does not equal 1")
        if(matmult(R,invR) == np.eye(3)).all:#Test to make sure R*invR=I 
            return invR
    except:
        print("Inverse cannot be calculated")


def VecToso3(omg):#Takes a 3-vector(angular velocity).
#Returns the skew symmetric matrix in so3.
    '''
Example Input: 
omg = [1,2,3]
Output:
[[0, -3, 2],
 [3, 0, -1],
 [-2, 1, 0]]
    '''
    if (len(omg)==3):
        return [[0,-omg[2],omg[1]], [omg[2],0,-omg[0]], [-omg[1],omg[0],0]]
    else:
        print ("Input vector is the wrong size")


def so3ToVec(so3mat):#Takes a 3x3 skew-symmetric matrix (an element of so(3)).
#Returns the corresponding vector (angular velocity).
    '''
Example Input: 
so3mat = [[0, -3, 2],
 [3, 0, -1],
 [-2, 1, 0]]
Output:
[1, 2, 3]
    '''
    try:
        if (so3mat[0][0]==0 and so3mat[1][1]==0 and so3mat[2][2]==0 and
        so3mat[0][1]==-so3mat[1][0] and
        so3mat[2][0]==-so3mat[0][2] and 
        so3mat[1][2]==-so3mat[2][1]):#Check if input is a skew-symmetric matrix
                return [so3mat[2][1],so3mat[0][2],so3mat[1][0]] #omg
        else:
            print ("Input is not a skew-symmetric matrix")
    except:
        print ("Input martix is the wrong shape")


def AxisAng3(expc3):#Takes A 3-vector of exponential coordinates for rotation.
#Returns unit rotation axis omghat and the corresponding rotation angle theta.
    '''
Example Input: 
expc3 = [1,2,3]
Output:
([0.2672612419124244, 0.5345224838248488, 0.8017837257372732], -->unit rotation axis omghat
 3.7416573867739413) -->rotation angle theta
    '''
    if (len(expc3)==3):
        theta = Magnitude(expc3)
        if (theta == 0):
            return ([0,0,0],0)
        else:
            return (Normalize(expc3), theta)#(omghat, theta)
    else:
        print ("Input vector is the wrong size")

    
def MatrixExp3(expc3):#Takes a 3-vector of exponential coordinates.
#Returns R (SO(3)) that is achieved by rotating about omghat by theta
#from an initial orientation R = I
#Rodriguez R = I + sin(theta)*omghat + (1-cos(theta))*omghat^2
    '''
Example Input: 
expc3 = [1,2,3]
Output:
[[-0.69492056,  0.71352099,  0.08929286],
[-0.19200697, -0.30378504,  0.93319235],
[ 0.69297817,  0.6313497 ,  0.34810748]]
    '''
    if (len(expc3)==3):
        omghat,theta = AxisAng3(expc3)
        return np.eye(3) + matmult(VecToso3(omghat),np.sin(theta)) + matmult(VecToso3(omghat),VecToso3(omghat))*(1-np.cos(theta))        
    else:
        print ("Input vector is the wrong size")


def MatrixLog3(R):#Takes R (rotation matrix).
#Returns the corresponding 3-vector of exponential coordinates (expc3 = omghat*theta).
    '''
Example Input: 
R = [[0, 0, 1],[1, 0, 0],[0, 1, 0]]

Output:
[1.2091995761561456, 1.2091995761561456, 1.2091995761561456]
    '''
    if (np.shape(R)==(3,3)):
        try:
            Rtrace = R[0][0]+R[1][1]+R[2][2]
            theta = np.arccos((Rtrace - 1)/2.0)
            omg = 1/(2*np.sin(theta))*np.array([R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]])
            if any(map(np.isinf, omg)) or any(map(np.isnan, omg)):
                theta = 0
                omg = 3*[1/np.sqrt(3)]
            return [omg[0]*theta,omg[1]*theta,omg[2]*theta]
        except:
            print("Matrix cannot be converted")        
    else:
        print ("Input matrix is the wrong size")


def RpToTrans (R,p):#Takes rotation matrix R and position p. 
#Returns corresponding homogeneous transformation matrix T SE(3)
    '''
Example Input: 
R = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
p = [1,2,5]
Output:
[[1, 0, 0, 1],
 [0, 0, -1, 2],
 [0, 1, 0, 5],
 [0, 0, 0, 1]]
    '''
    if (len(p)==3 and np.shape(R)==(3,3)):
        #Test for determinant = 1 +- 0.05 error
        if (det(R)*1.0>=0.95 and det(R)*1.0<=1.05):
            return [ [R[0][0],R[0][1],R[0][2],p[0]] , [R[1][0],R[1][1],R[1][2],p[1]], [R[2][0],R[2][1],R[2][2],p[2]] , [0,0,0,1]]
        print ("Input R is not a rotation matrix")
    else:
        print ("Input rotation matrix or position vector are the wrong size")


def TransToRp (T):#Takes transformation matrix T SE(3). 
#Returns R the corresponding rotation matrix,
#p the corresponding position vector.
    '''
Example Input: 
T = [[1,0,0,0],
     [0,0,-1,0],
     [0,1,0,3],
     [0,0,0,1]]
Output:
([[1, 0, 0], -->R
 [0, 0, -1], -->R
 [0, 1, 0]], -->R 
[0, 0, 3]) -->P
    '''
    if (np.shape(T)==(4,4)):
        R = [[T[0][0],T[0][1],T[0][2]],
                  [T[1][0],T[1][1],T[1][2]],
                  [T[2][0],T[2][1],T[2][2]]]
        p = [T[0][3],T[1][3],T[2][3]]
        #Test for determinant = 1 +- 0.05 error
        if (det(R)*1.0>=0.95 and det(R)*1.0<=1.05):
            return R,p
        print ("Input is not a transformation matrix")
    else:
        print ("Input Transformation matrix is the wrong size")


def TransInv(T):#Takes T a transformation matrix. 
#Returns its inverse.
#Uses the structure of transformation matrices to avoid taking a matrix inverse, for efficiency.
    '''
Example Input: 
T = [[1,0,0,0],
     [0,0,-1,0],
     [0,1,0,3],
     [0,0,0,1]]
Output:
[[1, 0, 0, 0],
 [0, 0, 1, -3],
 [0, -1, 0, 0],
 [0, 0, 0, 1]]
    '''
    try:
        R,p = TransToRp(T)
        Rt = RotInv(R)
        pt = np.dot(Rt, p)
        return [ [Rt[0][0],Rt[0][1],Rt[0][2],-pt[0]],
                 [Rt[1][0],Rt[1][1],Rt[1][2],-pt[1]],
                 [Rt[2][0],Rt[2][1],Rt[2][2],-pt[2]],
                 [0,0,0,1]]
    except:
        print ("Input matrix is not an element of SE3")


def VecTose3(V):#Takes a 6-vector (representing a spatial velocity). 
#Returns the corresponding 4x4 se(3) matrix.
    '''
Example Input: 
V = [1,2,3,4,5,6]
Output:
[[0, -3, 2, 4], [3, 0, -1, 5], [-2, 1, 0, 6], [0, 0, 0, 0]]
    '''
    if (len(V)==6):
        so3mat = VecToso3([V[0],V[1],V[2]])
        return [ [so3mat[0][0],so3mat[0][1],so3mat[0][2],V[3]] , [so3mat[1][0],so3mat[1][1],so3mat[1][2],V[4]], [so3mat[2][0],so3mat[2][1],so3mat[2][2],V[5]] , [0,0,0,0]]
    else:
        print ("Input vector is the wrong size")


def se3ToVec(se3mat):#Takes se3mat a 4x4 se(3) matrix.
#Returns the corresponding 6-vector (representing spatial velocity).
    '''
Example Input: 
se3mat = [[0, -3, 2, 4], [3, 0, -1, 5], [-2, 1, 0, 6], [0, 0, 0, 0]]
Output:
[1, 2, 3, 4, 5, 6]
    '''
    try:
        if (np.shape(se3mat)==(4,4)):
            return ([se3mat[2][1],se3mat[0][2],se3mat[1][0],
                 se3mat[0][3],se3mat[1][3],se3mat[2][3]])
        print ("Input matrix is the wrong size")
    except:
        print ("Input is not an element of se3")


def Adjoint(T):#Takes T a transformation matrix SE3 
#Returns the corresponding 6x6 adjoint representation [AdT]
    '''
Example Input: 
T = [[1,0,0,0], [0,0,-1,0], [0,1,0,3], [0,0,0,1]]
Output:
[[1, 0, 0, 0, 0, 0],
 [0, 0, -1, 0, 0, 0],
 [0, 1, 0, 0, 0, 0],
 [0, 0, 3, 1, 0, 0],
 [3, 0, 0, 0, 0, -1],
 [0, 0, 0, 0, 1, 0]]
    '''
    try:
        R,p = TransToRp(T)
        pp = VecToso3(p)
        Rt = matmult(pp,R) 
        return [[R[0][0],R[0][1],R[0][2],0,0,0],
                [R[1][0],R[1][1],R[1][2],0,0,0],
                 [R[2][0],R[2][1],R[2][2],0,0,0],
                  [Rt[0][0],Rt[0][1],Rt[0][2],R[0][0],R[0][1],R[0][2]],
                  [Rt[1][0],Rt[1][1],Rt[1][2],R[1][0],R[1][1],R[1][2]],
                  [Rt[2][0],Rt[2][1],Rt[2][2],R[2][0],R[2][1],R[2][2]]]
    except:
        print ("Input matrix is the wrong size")


def ScrewToAxis(q,s,h):
#Takes q: a point lying on the screw axis, 
#s: a unit vector in the direction of the screw axis,
#h: the pitch of the screw axis.
#Returns the corresponding normalized screw axis.
    '''
Example Input: 
q = [3,0,0]
s = [0,0,1]
h = 2
Output:
[[0], [0], [1], [0], [-3], [2]]
    '''
    if(len(q)==3 and len(s)==3 and isinstance(h, int)):
        sq=np.cross(s,q)
        return [[s[0]],
                [s[1]],
                [s[2]],
                [(-sq[0]+h*s[0])],
                [(-sq[1]+h*s[1])],
                [(-sq[2]+h*s[2])]]
    else:
        print("Inputs are the wrong size. -->  qE3, sE3, h scaler")


def AxisAng6(expc6):#Takes a 6-vector of exponential coordinates for rigid-body motion S*theta.
#Returns S: the corresponding normalized screw axis,
#theta: the distance traveled along/about S.
    '''
Example Input: 
expc6 = [1,0,0,1,2,3]
Output:
([1.0, 0.0, 0.0, 1.0, 2.0, 3.0], 1.0) --> First the srew axis and then the theta
    '''
    if (len(expc6)==6):
        theta = Magnitude([expc6[0],expc6[1],expc6[2]])
        if (theta == 0):
            theta = Magnitude([expc6[3],expc6[4],expc6[5]])
            if (theta==0):
                return ([0,0,0,0,0,0],0)
            return ([expc6[0]/theta*1.0,expc6[1]/theta*1.0,expc6[2]/theta*1.0,
                     expc6[3]/theta*1.0,expc6[4]/theta*1.0,expc6[5]/theta*1.0],
                    theta)#(S,theta)
        else:
            return ([expc6[0]/theta*1.0,expc6[1]/theta*1.0,expc6[2]/theta*1.0,
                     expc6[3]/theta*1.0,expc6[4]/theta*1.0,expc6[5]/theta*1.0],
                    theta)#(S,theta)
    else:
        print ("Input vector is the wrong size")


def MatrixExp6(expc6):#Takes a 6-vector of exponential coordinates (S*theta) 
#Returns a T matrix SE(3) that is achieved by traveling along/about the screw axis S
#for a distance theta from an initial configuration T = I
#Rodriguez R = I + sin(theta)*omg + (1-cos(theta))*omg^2
    '''
Example Input: 
expc6 = [1.5707963267948966, 0.0, 0.0, 0.0, 2.3561944901923448, 2.3561944901923457]
Output:
[[1.0, 0.0, 0.0, 0.0],
 [0.0, 0.0, -1.0, 0.0],
 [0.0, 1.0, 0.0, 3.0],
 [0, 0, 0, 1]]

    '''
    S,theta = AxisAng6(expc6)
    omg = [S[0],S[1],S[2]]
    if (Magnitude([expc6[0],expc6[1],expc6[2]])>0):
        UL = np.eye(3) + matmult(VecToso3(omg),np.sin(theta)) + matmult(VecToso3(omg),VecToso3(omg))*(1-np.cos(theta))
        UR = matmult(np.eye(3),theta) + matmult(VecToso3(omg),(1-np.cos(theta))) + matmult(VecToso3(omg),VecToso3(omg))*(theta-np.sin(theta))
        UR = np.dot(UR,[S[3],S[4],S[5]])
    else:
        UL = np.eye(3)
        UR = [expc6[3],expc6[4],expc6[5]]
    return [[UL[0][0],UL[0][1],UL[0][2],UR[0]],
            [UL[1][0],UL[1][1],UL[1][2],UR[1]],
             [UL[2][0],UL[2][1],UL[2][2],UR[2]],
              [0,0,0,1]]


def MatrixLog6(T):#Takes a transformation matrix T SE(3) 
#Returns the corresponding 6-vector of exponential coordinates S*theta
    '''
Example Input: 
T = [[1,0,0,0], [0,0,-1,0], [0,1,0,3], [0,0,0,1]]
Output:
[1.5707963267948966, 0.0, 0.0, 0.0, 2.3561944901923448, 2.3561944901923457]
    '''
    R,p = TransToRp(T)
    Rtrace = R[0][0]+R[1][1]+R[2][2]
    if(R==np.eye(3)).all():
        omg=[0,0,0]
        v=p
        theta=1
    
    else:
        if(Rtrace == -1):
            theta = pi
            omg = MatrixLog3(R)
            G = (1/theta)*np.eye(3) - 0.5*np.asarray(VecToso3(omg)) + ((1/theta)-((1/(tan(theta/2.0)))/2.0))*(matmult(VecToso3(omg),VecToso3(omg)))
            v = np.dot(G,p)

        else:
            theta = acos((Rtrace-1)/2.0)
            omg = so3ToVec((1/(2*np.sin(theta)))*(np.subtract(R, RotInv(R))))
            G = (1/theta)*np.eye(3) - 0.5*np.asarray(VecToso3(omg)) + ((1/theta)-((1/(tan(theta/2.0)))/2.0))*(matmult(VecToso3(omg),VecToso3(omg)))
            v = np.dot(G,p)     

    return ([omg[0]*theta,omg[1]*theta,omg[2]*theta,v[0]*theta,v[1]*theta,v[2]*theta])




'''
**********************************************************************************************
****************************  CHAPTER 4: FORWARD KINEMATICS  *********************************
**********************************************************************************************
'''
def FKinBody(M, Blist, thetalist):#Takes M: the home configuration (position and orientation) of the end-effector,
#Blist: The joint screw axes in the end-effector frame when the manipulator is at the home position,
#thetalist: A list of joint coordinates.
#Returns T (SE(3)) representing the end-effector frame
#when the joints are at the specified coordinates (i.t.o Body Frame).
    '''
Example Input: 
M = [[-1,0,0,0], [0,1,0,6], [0,0,-1,2], [0,0,0,1]]
Blist = [[0,0,-1,2,0,0],[0,0,0,0,1,0],[0,0,1,0,0,0.1]]
thetalist =[(pi/2.0),3,pi]
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
[  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
[ 0.          0.         -1.          1.68584073],
[ 0.  0.  0.  1.]]
    '''
    try:
        T = np.eye(4)
        T = np.dot(M,T)
        for i in range(len(Blist)):
            T = np.dot(T,MatrixExp6(np.asarray(Blist[i])*thetalist[i]))       
        
        return T
    except:
        print ("Input is not appropriate")


def FKinSpace(M, Slist, thetalist):#Takes M: the home configuration (position and orientation) of the end-effector,
#Slist: The joint screw axes in the space frame when the manipulator is at the home position,
#thetalist: A list of joint coordinates.
#Returns T (SE(3)) representing the end-effector frame 
#when the joints are at the specified coordinates (i.t.o Space Frame).
    '''
Example Input: 
M = [[-1,0,0,0], [0,1,0,6], [0,0,-1,2], [0,0,0,1]]
Slist = [[0,0,1,4,0,0],[0,0,0,0,1,0],[0,0,-1,-6,0,-0.1]]
thetalist =[(pi/2.0),3,pi]
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
[  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
[ 0.          0.         -1.          1.68584073],
[ 0.  0.  0.  1.]]
    '''
    try:
        T = np.eye(4)
        for i in range(len(Slist)):
            T = np.dot(T,MatrixExp6(np.asarray(Slist[i])*thetalist[i]))
        
        T = np.dot(T,M)
        return T
    except:
        print ("Input is not appropriate")





'''
**********************************************************************************************
**********************  CHAPTER 5: VELOCITY KINEMATICS AND STATICS  **************************
**********************************************************************************************
'''
def JacobianBody(Blist, thetalist):#Takes Blist: The joint screw axes in the end-effector frame when the manipulator is at the home position,
#thetalist: A list of joint coordinates. 
#Returns the corresponding body Jacobian (6xn real numbers).
    '''
Example Input: 
Blist = [[0,0,1,0,0.2,0.2],[1,0,0,2,0,3],[0,1,0,0,2,1],[1,0,0,0.2,0.3,0.4]]
thetalist = [0.2,1.1,0.1,1.2]
Output:
[[-0.04528405  0.99500417  0.          1.        ]
 [ 0.74359313  0.09304865  0.36235775  0.        ]
 [-0.66709716  0.03617541 -0.93203909  0.        ]
 [ 2.32586047  1.66809     0.56410831  0.2       ]
 [-1.44321167  2.94561275  1.43306521  0.3       ]
 [-2.06639565  1.82881722 -1.58868628  0.4       ]]
    '''
    if (len(Blist)==len(thetalist)):
        Jb = [[0]*6 for i in range(len(Blist))]
        for i in range(len(Blist)):
            if (len(Blist[i])==6):
                JbAdj = MatrixExp6(matmult(matmult(Blist[len(Blist)-1],-1),thetalist[len(Blist)-1])) 
                for j in range (len(Blist)-2-i):
                    JbAdj = matmult(JbAdj,MatrixExp6(matmult(matmult(Blist[len(Blist)-j-2],-1),thetalist[len(Blist)-j-2]))) 
               
                JbAdj = Adjoint(JbAdj)
                Jb[i] = matmult(JbAdj,Blist[i]) 
            else:
                print "There is the wrong number of elements in the inputted screw axes"
                return
        return np.asarray(Jb).T
    else:
        print "The input has a different number of screw axes and joint angles"
        return


def JacobianSpace(Slist, thetalist):#Takes Slist: The joint screw axes in the space frame when the manipulator is at the home position,
#thetalist: A list of joint coordinates.
#Returns the corresponding space Jacobian (6xn real numbers).
    '''
Example Input: 
Slist = [[0,0,1,0,0.2,0.2],[1,0,0,2,0,3],[0,1,0,0,2,1],[1,0,0,0.2,0.3,0.4]]
thetalist = [0.2,1.1,0.1,1.2]
Output:
[[ 0.          0.98006658 -0.09011564  0.95749426]
 [ 0.          0.19866933  0.4445544   0.28487557]
 [ 1.          0.          0.89120736 -0.04528405]
 [ 0.          1.95218638 -2.21635216 -0.51161537]
 [ 0.2         0.43654132 -2.43712573  2.77535713]
 [ 0.2         2.96026613  3.23573065  2.22512443]]
    '''
    if (len(Slist)==len(thetalist)):
        Js = [[0]*6 for i in range(len(Slist))]
        Js[0] = Slist[0]
        for i in range(1,len(Slist)):
            if (len(Slist[i])==6):               
                JsAdj = MatrixExp6(matmult(Slist[0],thetalist[0]))
                for j in range (i-1):                 
                    JsAdj = matmult(JsAdj, MatrixExp6(matmult(Slist[j+1],thetalist[j+1]))) 
                JsAdj = Adjoint(JsAdj)
                Js[i] = matmult(JsAdj,Slist[i]) 
            else:
                print "There is the wrong number of elements in the inputted screw axes"
                return
        return np.asarray(Js).T
    else:
        print "The input has a different number of screw axes and joint angles"
        return
    




'''
**********************************************************************************************
*****************************  CHAPTER 6: INVERSE KINEMATICS  ********************************
**********************************************************************************************
'''
def IKinBody(Blist, M, T, thetalist0, eomg, ev):
#Takes Blist: The joint screw axes in the end-effector frame 
#when the manipulator is at the home position.
#M: The home configuration of the end-effector.
#T: The desired end-effector configuration Tsd
#thetalist0: An initial guess of joint angles that are close to satisfying Tsd
#eomg: A small positive tolerance on the end-effector orientation error. The returned joint angles
#must give an end-effector orientation error less than eomg.
#ev: A small positive tolerance on the end-effector linear position error. The returned joint
#angles must give an end-effector position error less than ev.

#The maximum number of iterations before the algorithm is terminated has been hardcoded in as a variable called maxiterations. It is set to 20 at the start of the function, but can be changed if needed.  

#Returns thetalist: Joint angles that achieve T within the specified tolerances,
#success: A logical value where TRUE means that the function found a solution and FALSE
#means that it ran through the set number of maximum iterations without finding a solution
#within the tolerances eomg and ev.
#Uses an iterative Newton-Raphson root-finding method
    '''
Example Input: 
Blist = [[0,1,0,0.191,0,0.817],[0,0,1,0.095,-0.817,0],[0,0,1,0.095,-0.392,0],[0,0,1,0.095,0,0],[0,-1,0,-0.082,0,0],[0,0,1,0,0,0]]
M = [[1,0,0,-0.817],
    [0,0,-1,-0.191],
    [0,1,0,-0.006],
    [0,0,0,1]]
T = [[0,1,0,-0.6], [0,0,-1,0.1], [-1,0,0,0.1], [0,0,0,1]]
thetalist0 =[0,0,0,0,0,0]
eomg = 0.01
ev = 0.001
Output:
thetalist:
[-0.46921905 -0.83447622  1.39525223 -0.56107486 -0.46731326 -1.57056352]
success:
True
    '''
    maxiterations = 20
    success = False
    thf =[]#Final return variable
    thf.append(thetalist0)
    Vb = MatrixLog6(matmult(TransInv(FKinBody(M, Blist, thetalist0)),T))
    wb  = Magnitude ([Vb[0],Vb[1],Vb[2]])
    vb = Magnitude ([Vb[3],Vb[4],Vb[5]])
    for i in range (maxiterations):
        if (wb>eomg and vb>ev):
            thetalist0 = np.add(thetalist0, matmult(np.linalg.pinv(JacobianBody(Blist, thetalist0)),Vb))
            thf.append(thetalist0)
            Vb = MatrixLog6(matmult(TransInv(FKinBody(M, Blist, thetalist0)),T))
            wb  = Magnitude ([Vb[0],Vb[1],Vb[2]])
            vb = Magnitude ([Vb[3],Vb[4],Vb[5]])
        else:
            success = True
            return (thf[len(thf)-1],success)
    return (thf[len(thf)-1],success)


def IKinSpace(Slist, M, T, thetalist0, eomg, ev):
#Takes Slist: The joint screw axes in the space frame 
#when the manipulator is at the home position.
#M: The home configuration of the end-effector.
#T: The desired end-effector configuration Tsd
#thetalist0: An initial guess of joint angles that are close to satisfying Tsd
#eomg: A small positive tolerance on the end-effector orientation error. The returned joint angles
#must give an end-effector orientation error less than eomg.
#ev: A small positive tolerance on the end-effector linear position error. The returned joint
#angles must give an end-effector position error less than ev.

#The maximum number of iterations before the algorithm is terminated has been hardcoded in as a variable called maxiterations. It is set to 20 at the start of the function, but can be changed if needed.  

#Returns thetalist: Joint angles that achieve T within the specified tolerances,
#success: A logical value where TRUE means that the function found a solution and FALSE
#means that it ran through the set number of maximum iterations without finding a solution
#within the tolerances eomg and ev.
#Uses an iterative Newton-Raphson root-finding method
    '''
Example Input: 
Slist = [[0,0,1,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,1,0,-0.550,0,0.045],[0,0,1,0,0,0],[0,1,0,-0.850,0,0],[0,0,1,0,0,0]]
M = [[1,0,0,0],
    [0,1,0,0],
    [0,0,1,0.910],
    [0,0,0,1]]
T = [[1,0,0,0.4], [0,1,0,0], [0,0,1,0.4], [0,0,0,1]]
thetalist0 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]
eomg = 0.01
ev = 0.001
Output:
thetalist:
[6.93483863   7.12127381  -17.44241224  -20.55900553  -8.60119276  17.30667641  13.69803296]
success:
True
    '''
    maxiterations = 20
    success = False
    thf =[]#Final return variable
    thf.append(thetalist0)
    Vs = MatrixLog6(matmult(TransInv(FKinSpace(M, Slist, thetalist0)),T))
    wb  = Magnitude ([Vs[0],Vs[1],Vs[2]])
    vb = Magnitude ([Vs[3],Vs[4],Vs[5]])
    for i in range (maxiterations):
        if (wb>eomg and vb>ev):
            Jb = matmult(Adjoint(TransInv(FKinSpace(M, Slist, thetalist0))),JacobianSpace(Slist, thetalist0))
            thetalist0 = np.add(thetalist0, matmult(np.linalg.pinv(Jb),Vs))
            thf.append(thetalist0)
            Vs = MatrixLog6(matmult(TransInv(FKinSpace(M, Slist, thetalist0)),T))
            wb  = Magnitude ([Vs[0],Vs[1],Vs[2]])
            vb = Magnitude ([Vs[3],Vs[4],Vs[5]])
        else:
            success = True
            print i
            return (thf[len(thf)-1],success)
    return (thf[len(thf)-1],success)





'''
**********************************************************************************************
***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
**********************************************************************************************
'''
def ad(V):#Takes 6-vector spatial velocity 
#Returns the corresponding 6x6 matrix [adV].
#Used to calculate the Lie bracket [V1, V2] = [adV1]V2
    '''
Example Input: 
V = [1,2,3,4,5,6]
Output:
[[0, -3, 2, 0, 0, 0],
 [3, 0, -1, 0, 0, 0],
 [-2, 1, 0, 0, 0, 0],
 [0, -6, 5, 0, -3, 2],
 [6, 0, -4, 3, 0, -1],
 [-5, 4, 0, -2, 1, 0]]
    '''
    w = VecToso3([V[0],V[1],V[2]])
    v = VecToso3([V[3],V[4],V[5]])
    adV = [[w[0][0],w[0][1],w[0][2],0,0,0],[w[1][0],w[1][1],w[1][2],0,0,0],[w[2][0],w[2][1],w[2][2],0,0,0],[v[0][0],v[0][1],v[0][2],w[0][0],w[0][1],w[0][2]],[v[1][0],v[1][1],v[1][2],w[1][0],w[1][1],w[1][2]],[v[2][0],v[2][1],v[2][2],w[2][0],w[2][1],w[2][2]]]
    return adV

    
def InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist):
#Takes thetalist: n-vector of joint variables,
#dthetalist: n-vector of joint rates,
#ddthetalist: n-vector of joint accelerations,
#g: Gravity vector g,
#Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
#Mlist: List of link frames {i} relative to {i-1} at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns taulist: The n-vector of required joint forces/torques.
#This function uses forward-backward Newton-Euler iterations to solve the equation:
#taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) + g(thetalist) + Jtr(thetalist)Ftip
    '''
Example Input (3 Link Robot):
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]
ddthetalist = [2,1.5,1]

g = [0,0,-9.8]
Ftip = [1,1,1,1,1,1]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[12.287409365123651, -20.555491300883507, -2.9403622930411073]
    '''
    #******INITIALISATION********
    n=len(Mlist)
    Mi = [Mlist[0]]
    Ai = [matmult(Adjoint(TransInv(Mi[0])),Slist[0])]
    Ti = [matmult(Mlist[0],MatrixExp6(matmult(Ai[0],thetalist[0])))]
    Vi=[]
    Vi.append(matmult(Adjoint(TransInv(Ti[0])),[0,0,0,0,0,0])+matmult(Ai[0],dthetalist[0]))
    Vdi =[]
    Vdi.append(matmult(Adjoint(TransInv(Ti[0])),[0,0,0,-g[0],-g[1],-g[2]])+matmult(matmult(ad(Vi[0]),Ai[0]),dthetalist[0])+matmult(Ai[0],ddthetalist[0]))
    #****************************
    #*****Forward Iteration******
    for i in range (1,n):
        Mi.append(matmult(Mi[i-1],Mlist[i]))
        Ai.append(matmult(Adjoint(TransInv(Mi[i])),Slist[i]))
        Ti.append(matmult(Mlist[i],MatrixExp6(matmult(Ai[i],thetalist[i]))))
        Vi.append(matmult(Adjoint(TransInv(Ti[i])),Vi[i-1])+matmult(Ai[i],dthetalist[i]))
        Vdi.append(matmult(Adjoint(TransInv(Ti[i])),Vdi[i-1])+matmult(matmult(ad(Vi[i]),Ai[i]),dthetalist[i])+matmult(Ai[i],ddthetalist[i]))
    #****************************
   
    #******INITIALISATION********
    Fi =[[None]]*(n)
    Fi[n-1] = matmult(np.array(Adjoint(TransInv(Ti[n-1]))).T,Ftip)+matmult(Glist[n-1],Vdi[n-1])-matmult(np.array(ad(Vi[n-1])).T,matmult(Vi[n-1],Glist[n-1]))
    taulist = [[None]]*(n)
    taulist[n-1]=matmult(np.array(Fi[n-1]).T,Ai[n-1])
    #****************************
    #*****Backward Iteration*****
    for i in range (n,1,-1):
        Fi[i-2] = matmult(np.array(Adjoint(TransInv(Ti[i-2]))).T,Ftip)+matmult(Glist[i-2],Vdi[i-2])-matmult(np.array(ad(Vi[i-2])).T,matmult(Vi[i-2],Glist[i-2]))
        taulist[i-2]=matmult(np.array(Fi[i-2]).T,Ai[i-2])
    #****************************

    return taulist


def MassMatrix(thetalist, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns M: The numerical inertia matrix M(thetalist) of an n-joint serial chain at the
#given configuration thetalist.
#This function calls InverseDynamics n times, each time passing a ddthetalist vector
#with a single element equal to one and all other inputs set to zero. 
#Each call of InverseDynamics generates a single column,
#and these columns are assembled to create the inertia matrix.
    '''
Example Input (3 Link Robot):
thetalist = [0.1,0.1,0.1]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[[3.0865629109397004, -0.28598437110125424, -0.0071842639094406753],
[0.0, 0.88490141218343321, 0.43215736829319362],
 [0.0, 0.0, 0.19163085751427505]]

    '''
    n = len(Mlist)
    dthetalist = [0]*n
    g = (0,0,0)
    Ftip = [0]*6
    M = []
    for i in range (n):
        ddthetalist = [0]*n
        ddthetalist[i]=1
        M.append(InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist))
    
    return M


def VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#dthetalist: A list of joint rates,
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns c: The vector c(thetalist,dthetalist) of Coriolis and centripetal terms
#for a given thetalist and dthetalist.
#This function calls InverseDynamics with g = 0, Ftip = 0, and ddthetalist = 0.
    '''
Example Input (3 Link Robot):
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[0.0, -0.027703940237666886, -0.0068913200682489129]
    '''
    n = len(Mlist)
    g = (0,0,0)
    Ftip = [0]*6
    ddthetalist = [0]*n
    c = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    
    return c


def GravityForces(thetalist, g, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#g: 3-vector for gravitational acceleration,
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns grav: The joint forces/torques required to overcome gravity at thetalist
#This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and ddthetalist = 0.
    '''
Example Inputs (3 Link Robot):
thetalist = [0.1,0.1,0.1]

g = [0,0,-9.8]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[3.291711438237281, -22.813661135010662, -5.4415891999683605]
    '''
    n = len(Mlist)
    dthetalist = [0]*n
    Ftip = [0]*6
    ddthetalist = [0]*n
    grav = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)

    return grav


def EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns JTFtip: The joint forces and torques required only to create the end-effector force Ftip.
#This function calls InverseDynamics with g = 0, dthetalist = 0, and ddthetalist = 0.
    '''
Example Input (3 Link Robot):
thetalist = [0.1,0.1,0.1]

Ftip = [1,1,1,1,1,1]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[2.8225721050069685, 1.5304903982921769, 1.6826198448603173]
    '''
    n = len(Mlist)
    dthetalist = [0]*n
    ddthetalist = [0]*n
    g = [0,0,0]
    JTFtip = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    return JTFtip


def ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist):
#Takes thetalist: A list of joint variables,
#dthetalist: A list of joint rates,
#taulist: An n-vector of joint forces/torques,
#g: Gravity vector g,
#Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns ddthetalist: The resulting joint accelerations.
#This function computes ddthetalist by solving:
#Mlist(thetalist)ddthetalist = taulist - c(thetalist,dthetalist) - g(thetalist) - Jtr(thetalist)Ftip
    '''
Example Input (3 Link Robot):
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]

taulist = [0.5, 0.6, 0.7]

g = [0,0,-9.8]
Ftip = [1,1,1,1,1,1]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))

Output:
[-17.32883936,  20.99456129,  10.36507714]
    '''
    RHS = (np.subtract(taulist,VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist))-GravityForces(thetalist, g, Mlist, Glist, Slist)-EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist))
    InMatTrans = np.array(MassMatrix(thetalist, Mlist, Glist, Slist)).T
    ddthetalist =  matmult(InMatTrans,RHS)
    return ddthetalist


def EulerStep(thetalist,dthetalist,ddthetalist,dt):
#Takes thetalist: n-vector of joint variables,
#dthetalist: n-vector of joint rates,
#ddthetalist: n-vector of joint accelerations,
#dt: The timestep delta t

#Returns thetalistNext: Vector of joint variables after dt from first order Euler integration,
#dthetalistNext: Vector of joint rates after dt from first order Euler integration.
    '''
Example Inputs (3 Link Robot):
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]
ddthetalist = [2,1.5,1]
dt = 0.1

Output:
thetalistNext:
[ 0.11,  0.12,  0.13]
dthetalistNext:
[ 0.3 ,  0.35,  0.4 ]
    '''
    thetalistNext = thetalist+matmult(dt,dthetalist)
    dthetalistNext = dthetalist+matmult(dt,ddthetalist)
    return thetalistNext, dthetalistNext


def InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, Ftipmat, Mlist, Glist, Slist):
#Takes thetamat: An N x n matrix of robot joint variables,
#dthetamat: An N x n matrix of robot joint velocities,
#ddthetamat: An N x n matrix of robot joint accelerations,
#g: Gravity vector g,
#Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (If there are no tip
#forces the user should input a zero and a zero matrix will be used),
#Mlist: List of link frames i relative to i-1 at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame.

#Returns taumat: The N x n matrix of joint forces/torques for the specified trajectory, where each of
#the N rows is the vector of joint forces/torques at each time step.
#This function uses InverseDynamics to calculate the joint forces/torques required to move the
#serial chain along the given trajectory.
    '''
#Example Inputs (3 Link Robot):
from math import pi
import numpy as np
import robot_calc_functions as r
import matplotlib.pyplot as plt
#Create a trajectory to follow using functions from Chapter 9
thetastart =[0,0,0]
thetaend =[pi/2,pi/2,pi/2]
Tf = 3
N= 1000
method = 5 
traj = r.JointTrajectory(thetastart, thetaend, Tf, N, method)
thetamat = []
dthetamat = []
ddthetamat = []
dt = Tf/(N-1.0)
for i in range(0,len(traj)):
    thetamat.append(traj[i])
    dthetamat.append((thetamat[i]-thetamat[i-1])/dt)
    ddthetamat.append((dthetamat[i]-dthetamat[i-1])/dt)
#Initialise robot descripstion (Example with 3 links)
g = [0,0,-9.8]
Ftipmat = [[1,1,1,1,1,1]]*N
M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T
Mlist = np.array((M01,M12,M23))
G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))
Glist = np.array((G1,G2,G3))
Slist = np.array(([1.,0.,1.,0.,0.23,0.1],[0.,1.,1.,-.089,0.,0.2],[0.,1.,0.,-.089,0.,.425]))
taumat = r.InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, Ftipmat, Mlist, Glist, Slist)

#Output using matplotlib to plot the joint forces/torques
Tau1 = []
Tau2 = []
Tau3 = []

for i in range(0,len(taumat)):
    Tau1.append(taumat[i][0])
    Tau2.append(taumat[i][1])
    Tau3.append(taumat[i][2])
timestamp = np.linspace(0,Tf,N)
plt.plot(timestamp, Tau1, label = "Tau1")
plt.plot(timestamp, Tau2, label = "Tau2")
plt.plot(timestamp, Tau3, label = "Tau3")
plt.ylim (-12,12)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Torque")
plt.title("Plot of Torque Trajectories")
plt.show()
    '''
    taumat = []
    if Ftipmat == 0:
        NewFtipmat = [[0,0,0,0,0,0]]*len(thetamat)
    else:
        NewFtipmat = Ftipmat;
    for i in range(len(thetamat)):
        taumat.append(InverseDynamics(thetamat[i], dthetamat[i], ddthetamat[i], g, NewFtipmat[i], Mlist, Glist, Slist))
    return taumat


def ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes):
#Takes thetalist: n-vector of initial joint variables,
#dthetalist: n-vector of initial joint rates,
#taumat: An N x n matrix of joint forces/torques, where each row is the joint effort at any time step,
#g: Gravity vector g,
#Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (If there are no tip
#forces the user should input a zero and a zero matrix will be used),
#Mlist: List of link frames {i} relative to {i-1} at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame,
#dt: The timestep between consecutive joint forces/torques,
#intRes: Integration resolution is the number of times integration (Euler) takes places
#between each time step. Must be an integer value greater than or equal to 1

#Returns thetamat: The N x n matrix of robot joint angles resulting from the 
#specified joint forces/torques,
#dthetamat: The N x n matrix of robot joint velocities.
#This function simulates the motion of a serial chain given an open-loop history 
#of joint forces/torques.
#It calls a numerical integration procedure that uses ForwardDynamics.
    '''
#Example Inputs (3 Link Robot):
from math import pi
import numpy as np
import robot_calc_functions as r
import matplotlib.pyplot as plt
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]
taumat = [[3.63,-6.58,-5.57], [3.74,-5.55,-5.5], [4.31,-0.68,-5.19],[5.18,5.63,-4.31],[5.85,8.17,-2.59],[5.78,2.79,-1.7 ],[4.99,-5.3 ,-1.19],[4.08,-9.41,0.07],[3.56,-10.1,0.97],[3.49,-9.41,1.23]]

#Initialise robot description (Example with 3 links)
g = [0,0,-9.8]
Ftipmat = [[1,1,1,1,1,1]]*len(taumat)
M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T
G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))
Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))
dt = 0.1
intRes = 8

thetamat, dthetamat = r.ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes)

#Output using matplotlib to plot the joint angle/velocities
import matplotlib.pyplot as plt
theta1 = []
theta2 = []
theta3 = []
dtheta1 = []
dtheta2 = []
dtheta3 = []
for i in range(0,len(thetamat)):
    theta1.append(thetamat[i][0])
    theta2.append(thetamat[i][1])
    theta3.append(thetamat[i][2])
    dtheta1.append(dthetamat[i][0])
    dtheta2.append(dthetamat[i][1])
    dtheta3.append(dthetamat[i][2])
N = len(thetamat)
Tf = len(taumat)*dt
timestamp = np.linspace(0,Tf,N)
plt.plot(timestamp, theta1, label = "Theta1")
plt.plot(timestamp, theta2, label = "Theta2")
plt.plot(timestamp, theta3, label = "Theta3")
plt.plot(timestamp, dtheta1, label = "DTheta1")
plt.plot(timestamp, dtheta2, label = "DTheta2")
plt.plot(timestamp, dtheta3, label = "DTheta3")
plt.ylim (-10,10)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Joint Angles/Velocities")
plt.title("Plot of Joint Angles and Joint Velocities")
plt.show()
    '''
    if intRes < 1 or isinstance(intRes, float):
        print "Integration resolution must be an integer value greater than zero."
        return
    if Ftipmat == 0:
        NewFtipmat = [[0,0,0,0,0,0]]*len(taumat)
    else:
        NewFtipmat = Ftipmat;
    thetamat = []
    thetamat.append(thetalist)
    dthetamat = []
    dthetamat.append(dthetalist)
    for i in range(len(taumat)):        
        for j in range(intRes):
            ddthetalist = ForwardDynamics(thetalist, dthetalist, taumat[i], g, NewFtipmat[i], Mlist, Glist, Slist)
            thetalist,dthetalist = EulerStep(thetalist,dthetalist,ddthetalist,(dt/intRes))
            thetamat.append(thetalist)
            dthetamat.append(dthetalist)
    return thetamat, dthetamat







'''
**********************************************************************************************
****************************  CHAPTER 9: TRAJECTORY GENERATION  ******************************
**********************************************************************************************
'''
def CubicTimeScaling(Tf, t):
#Takes Tf: Total time of the motion in seconds from rest to rest,
#t: The current time t satisfying 0<t<Tf.

#Returns s: The path parameter s(t) corresponding to a third-order polynomial motion that begins and
#ends at zero velocity.
    '''
Example Input: 
Tf = 2
t = 0.6
Output:
0.21600000000000003
    '''
    if (t > Tf or t < 0):
        print ("Input is not appropriate.")
        return
    #a0 and a1 are zero
    #Setting a2 and a3
    Tf = Tf + 0.0
    t = t + 0.0
    a2 = (3/(Tf**2))
    a3 = (-2/(Tf**3))
    s = ((a2*(t**2))+(a3*(t**3)))
    return s


def QuinticTimeScaling(Tf, t):
#Takes Tf: Total time of the motion in seconds from rest to rest,
#t: The current time t satisfying 0<t<Tf.

#Returns s: The path parameter s(t) corresponding to a fifth-order polynomial motion that begins and
#ends at zero velocity and zero acceleration.
    '''
Example Input: 
Tf = 2
t = 0.6
Output:
0.16307999999999995
    '''
    if (t > Tf or t < 0):
        print ("Input is not appropriate.")
        return
    #a0, a1, and a2 are zero
    #Setting a3, a4, and a5
    Tf = Tf+0.0
    t = t + 0.0
    a3 = (10/(Tf**3))
    a4 = (-15/(Tf**4))
    a5 = (6/(Tf**5))
    s = ((a3*(t**3))+(a4*(t**4))+(a5*(t**5)))
    return s


def JointTrajectory(thetastart, thetaend, Tf, N, method):
#Takes thetastart: The initial joint variables,
#thetaend: The final joint variables,
#Tf: Total time of the motion in seconds from rest to rest,
#N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory,
#method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling
#and 5 indicates quintic (fifth-order polynomial) time scaling.

#Returns traj: A trajectory as an N x n matrix, where each row is an n-vector of joint variables at an
#instant in time. The first row is thetastart and the Nth row is thetaend . The elapsed time between
#each row is Tf/(N - 1).
#The returned trajectory is a straight-line motion in joint space.
#Animation example can be seen at https://www.youtube.com/watch?v=fVElSuS1GgI
    '''
Example Input: 
thetastart = [1,0,0,1,1,0.2,0,1]
thetaend = [1.2,0.5,0.6,1.1,2,2,0.9,1]
Tf = 4
N = 6
method = 3
Output:
[[ 1.    ,  0.    ,  0.    ,  1.    ,  1.    ,  0.2   ,  0.    ,  1.    ],
 [ 1.0208,  0.052 ,  0.0624,  1.0104,  1.104 ,  0.3872,  0.0936,  1.    ],
 [ 1.0704,  0.176 ,  0.2112,  1.0352,  1.352 ,  0.8336,  0.3168,  1.    ],
 [ 1.1296,  0.324 ,  0.3888,  1.0648,  1.648 ,  1.3664,  0.5832,  1.    ],
 [ 1.1792,  0.448 ,  0.5376,  1.0896,  1.896 ,  1.8128,  0.8064,  1.    ],
 [ 1.2   ,  0.5   ,  0.6   ,  1.1   ,  2.    ,  2.    ,  0.9   ,  1.    ]]
    '''
    #Check required method
    if (method != 3 and method != 5):
            print "Please input either 3 or 5 for the time scaling method"
            return
    #Check that N has enough points
    if (N < 2):
            print "Please input a bigger N"
            return
    #Check that thestart and thetaend are the same size
    if (len(thetastart)!=len(thetaend)):
            print "Please enter a theta start the same size as your theta end"
            return
    #Convert to floats
    Tf += 0.0
    N += 0.0

    timegap = Tf/(N-1)#difference in time between each point
    traj=[]#empty array to store joint angles
    
    curr_time=0
    while(curr_time<=Tf):
        if(method==3):#Cubic
            traj.append(np.add(matmult(1-CubicTimeScaling(Tf, curr_time), thetastart),(matmult(CubicTimeScaling(Tf, curr_time),thetaend))))
        else:#Quintic
            traj.append(np.add(matmult(1-QuinticTimeScaling(Tf, curr_time), thetastart),(matmult(QuinticTimeScaling(Tf, curr_time),thetaend))))
        curr_time += timegap
            
    traj = np.asarray(traj)
    return traj


def ScrewTrajectory(Xstart, Xend, Tf, N, method):
#Takes Xstart: The initial end-effector configuration,
#Xend: The final end-effector configuration,
#Tf: Total time of the motion in seconds from rest to rest,
#N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory,
#method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling
#and 5 indicates quintic (fifth-order polynomial) time scaling.

#Returns traj: The discretized trajectory as a list of N matrices in SE(3) separated in 
#time by Tf/(N-1). The first in the list is Xstart and the Nth is Xend .
#This function calculates a trajectory corresponding to the screw motion about a space screw axis.
    '''
Example Input: 
Xstart = [[1,0,0,1],[0,1,0,0],[0,0,1,1],[0,0,0,1]]
Xend = [[0,0,1,0.1],[1,0,0,0],[0,1,0,4.1],[0,0,0,1]]
Tf = 5
N = 4
method = 3
Output:
[[[ 1.     0.     0.     1.   ]
  [ 0.     1.     0.     0.   ]
  [ 0.     0.     1.     1.   ]
  [ 0.     0.     0.     1.   ]]

 [[ 0.904 -0.25   0.346  0.441]
  [ 0.346  0.904 -0.25   0.529]
  [-0.25   0.346  0.904  1.601]
  [ 0.     0.     0.     1.   ]]

 [[ 0.346 -0.25   0.904 -0.117]
  [ 0.904  0.346 -0.25   0.473]
  [-0.25   0.904  0.346  3.274]
  [ 0.     0.     0.     1.   ]]

 [[-0.     0.     1.     0.1  ]
  [ 1.    -0.     0.    -0.   ]
  [ 0.     1.    -0.     4.1  ]
  [ 0.     0.     0.     1.   ]]]
    '''
    #Check required method
    if (method != 3 and method != 5):
            print "Please input either 3 or 5 for the time scaling method"
            return
    #Check that N has enough points
    if (N < 2):
            print "Please input a bigger N"
            return
    #Check that Xstart and Xend are the same size
    if (len(Xstart)!=len(Xend)):
            print "Please enter a Xstart the same size as your Xend"
            return

    #Convert to floats
    Tf += 0.0
    N += 0.0

    timegap = Tf/(N-1)#difference in time between each point
    traj=[]#empty array to store trajectory

    curr_time=0
    while(curr_time<=Tf):
        if(method==3):#Cubic
            traj.append(matmult(Xstart,MatrixExp6(matmult(MatrixLog6(matmult(TransInv(Xstart),Xend)),CubicTimeScaling(Tf, curr_time)))))
        else:#Quintic
            traj.append(matmult(Xstart,MatrixExp6(matmult(MatrixLog6(matmult(TransInv(Xstart),Xend)),QuinticTimeScaling(Tf, curr_time)))))
        curr_time += timegap
            
    traj = np.asarray(traj)
    return traj


def CartesianTrajectory(Xstart, Xend, Tf, N, method):
#Takes Xstart: The initial end-effector configuration,
#Xend: The final end-effector configuration,
#Tf: Total time of the motion in seconds from rest to rest,
#N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory,
#method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling
#and 5 indicates quintic (fifth-order polynomial) time scaling.

#Returns traj: The discretized trajectory as a list of N matrices in SE(3) separated in 
#time by Tf/(N-1). The first in the list is Xstart and the Nth is Xend .
#This function is Similar to ScrewTrajectory, except the origin of the end-effector frame
#follows a straight line, decoupled from the rotational motion.
#Animation example can be seen at https://www.youtube.com/watch?v=ycaGRk_0AE8
    '''
Example Input: 
Xstart = [[1,0,0,1],[0,1,0,0],[0,0,1,1],[0,0,0,1]]
Xend = [[0,0,1,0.1],[1,0,0,0],[0,1,0,4.1],[0,0,0,1]]
Tf = 5
N = 4
method = 5
Output:
[[[ 1.     0.     0.     1.   ]
  [ 0.     1.     0.     0.   ]
  [ 0.     0.     1.     1.   ]
  [ 0.     0.     0.     1.   ]]

 [[ 0.937 -0.214  0.277  0.811]
  [ 0.277  0.937 -0.214  0.   ]
  [-0.214  0.277  0.937  1.651]
  [ 0.     0.     0.     1.   ]]

 [[ 0.277 -0.214  0.937  0.289]
  [ 0.937  0.277 -0.214  0.   ]
  [-0.214  0.937  0.277  3.449]
  [ 0.     0.     0.     1.   ]]

 [[-0.     0.     1.     0.1  ]
  [ 1.    -0.     0.     0.   ]
  [ 0.     1.    -0.     4.1  ]
  [ 0.     0.     0.     1.   ]]]
    '''
    #Check required method
    if (method != 3 and method != 5):
            print "Please input either 3 or 5 for the time scaling method"
            return
    #Check that N has enough points
    if (N < 2):
            print "Please input a bigger N"
            return
    #Check that Xstart and Xend are the same size
    if (len(Xstart)!=len(Xend)):
            print "Please enter a Xstart the same size as your Xend"
            return

    #Convert to floats
    Tf += 0.0
    N += 0.0

    timegap = Tf/(N-1)#difference in time between each point
    traj=[]#empty array to store trajectory

    #Separate the Rotational motion and the straight line
    Rstart, Pstart = TransToRp(Xstart)
    Rend, Pend = TransToRp(Xend)

    curr_time=0
    while(curr_time<=Tf):
        if(method==3):#Cubic
            Pcurrent = np.add(matmult((1-CubicTimeScaling(Tf, curr_time)),Pstart), (matmult(CubicTimeScaling(Tf,curr_time),Pend)))
            Rcurrent = (matmult(Rstart,MatrixExp3(matmult(MatrixLog3(matmult(RotInv(Rstart),Rend)),CubicTimeScaling(Tf, curr_time)))))            
        else:#Quintic
            Pcurrent = np.add(matmult(1-QuinticTimeScaling(Tf, curr_time), Pstart),(matmult(QuinticTimeScaling(Tf, curr_time),Pend)))
            Rcurrent = (matmult(Rstart,MatrixExp3(matmult(MatrixLog3(matmult(RotInv(Rstart),Rend)),QuinticTimeScaling(Tf, curr_time)))))
        
        #Put R and P back together and add it to the matrix
        traj.append(RpToTrans(Rcurrent, Pcurrent))
        curr_time += timegap
            
    traj = np.asarray(traj)
    return traj





'''
**********************************************************************************************
********************************  CHAPTER 11: ROBOT CONTROL  *********************************
**********************************************************************************************
'''
def ComputedTorque(thetalist,dthetalist,eint,g,Mlist,Glist,Slist,thetalistd,dthetalistd,ddthetalistd,Kp,Ki,Kd):
#Takes thetalist: n-vector of joint variables,
#dthetalist: n-vector of joint rates,
#eint: n-vector of the time-integral of joint errors,
#g: Gravity vector g,
#Mlist: List of link frames {i} relative to {i-1} at the home position,
#Glist: Spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame,
#thetalistd: n-vector of reference joint variables,
#dthetalistd: n-vector of reference joint velocities,
#ddthetalistd: n-vector of reference joint accelerations,
#Kp: The feedback proportional gain (identical for each joint),
#Ki: The feedback integral gain (identical for each joint),
#Kd: The feedback derivative gain (identical for each joint).

#Returns taulist: The vector of joint forces/torques computed by the 
#feedback linearizing controller at the current instant.
    '''
Example Input: 
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]
eint = [0.2,0.2,0.2]
g = [0,0,-9.8]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,0.23,0.1],[0.,1.,1.,-.089,0.,0.2],[0.,1.,0.,-.089,0.,.425]))

thetalistd = [1,1,1]
dthetalistd = [2,1.2,2]
ddthetalistd = [0.1,0.1,0.1]

Kp = 1.3
Ki = 1.2
Kd = 1.1

Output:
[ 7.44175458  1.32493661 -4.81808147]
    '''
    e = np.subtract(thetalistd,thetalist)
    eder = np.subtract(dthetalistd,dthetalist)

    M = MassMatrix(thetalist,Mlist,Glist,Slist)
    c = VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
    grav = GravityForces(thetalist,g,Mlist,Glist,Slist)

    taulist = matmult(M,((ddthetalistd + matmult(Kp,e) + matmult(Ki,eint) + matmult(Kd,eder)))) + c + grav

    return taulist


def SimulateControl(thetalist,dthetalist,g,Ftipmat,Mlist,Glist,Slist,thetamatd,dthetamatd,ddthetamatd,gtilde,Mtildelist,Gtildelist,Kp,Ki,Kd,dt,intRes):
#Takes thetalist: n-vector of initial joint variables,
#dthetalist: n-vector of initial joint velocities,
#g: Actual gravity vector g,
#Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (If there are no tip
#forces the user should input a zero and a zero matrix will be used),
#Mlist: Actual list of link frames i relative to i-1 at the home position,
#Glist: Actual spatial inertia matrices Gi of the links,
#Slist: Screw axes Si of the joints in a space frame,
#thetamatd: An Nxn matrix of desired joint variables from the reference trajectory,
#dthetamatd: An Nxn matrix of desired joint velocities,
#ddthetamatd: An Nxn matrix of desired joint accelerations,
#gtilde: The (possibly incorrect) model of the gravity vector,
#Mtildelist: The (possibly incorrect) model of the link frame locations,
#Gtildelist: The (possibly incorrect) model of the link spatial inertias,
#Kp: The feedback proportional gain (identical for each joint),
#Ki: The feedback integral gain (identical for each joint),
#Kd: The feedback derivative gain (identical for each joint),
#dt: The timestep between points on the reference trajectory.
#intRes: Integration resolution is the number of times integration (Euler) takes places
#between each time step. Must be an integer value greater than or equal to 1.

#Returns taumat: An Nxn matrix of the controllers commanded joint forces/torques, where each row
#of n forces/torques corresponds to a single time instant,
#thetamat: An Nxn matrix of actual joint angles.
#The end of this function plots all the actual and desired joint angles using
#matplotlib and random libraries.
    '''
#Example Input: 
from math import pi
import numpy as np
import robot_calc_functions as r
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]

#Initialise robot description (Example with 3 links)
g = [0,0,-9.8]

M01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.089159,1.])).T
M12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.28,.13585,0.,1.])).T
M23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.1197,.395,1])).T

G1 = np.array(([.010267,0.,0.,0.,0.,0.],[0.,.010267,0.,0.,0.,0.],[0.,0.,.00666,0.,0.,0.],[0.,0.,0.,3.7,0.,0.],[0.,0.,0.,0.,3.7,0.],[0.,0.,0.,0.,0.,3.7]))
G2 = np.array(([.22689,0.,0.,0.,0.,0.],[0.,.22689,0.,0.,0.,0.],[0.,0.,.0151074,0.,0.,0.],[0.,0.,0.,8.393,0.,0.],[0.,0.,0.,0.,8.393,0.],[0.,0.,0.,0.,0.,8.393]))
G3 = np.array(([.0494433,0.,0.,0.,0.,0.],[0.,.0494433,0.,0.,0.,0.],[0.,0.,.004095,0.,0.,0.],[0.,0.,0.,2.275,0.,0.],[0.,0.,0.,0.,2.275,0.],[0.,0.,0.,0.,0.,2.275]))

Glist = np.array((G1,G2,G3))
Mlist = np.array((M01,M12,M23))

Slist = np.array(([1.,0.,1.,0.,1.,0.],[0.,1.,0.,-.089,0.,0.],[0.,1.,0.,-.089,0.,.425]))
dt = 0.01

#Create a trajectory to follow
thetaend =[pi/2,pi,1.5*pi]
Tf = 1
N = (Tf/dt)

method = 5
traj = r.JointTrajectory(thetalist, thetaend, Tf, N, method)
thetamatd = [thetalist]
dthetamatd = [dthetalist]
ddthetamatd = [0,0,0]

for i in range(1,len(traj)):
    thetamatd.append(traj[i])
    dthetamatd.append((thetamatd[i]-thetamatd[i-1])/dt)
    ddthetamatd.append((dthetamatd[i]-dthetamatd[i-1])/dt)

#Possibly wrong robot description (Example with 3 links)
gtilde = [0.8,0.2,-8.8]

Mhat01 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,.1,1.])).T
Mhat12 = np.array(([0.,0.,-1.,0.],[0.,1.,0.,0.],[1.,0.,0.,0.],[.3,.2,0.,1.])).T
Mhat23 = np.array(([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,-.2,.4,1])).T

Gtilde1 = np.array(([.1,0.,0.,0.,0.,0.],[0.,.1,0.,0.,0.,0.],[0.,0.,.1,0.,0.,0.],[0.,0.,0.,4.0,0.,0.],[0.,0.,0.,0.,4.0,0.],[0.,0.,0.,0.,0.,3.0]))
Gtilde2 = np.array(([.3,0.,0.,0.,0.,0.],[0.,.3,0.,0.,0.,0.],[0.,0.,.1,0.,0.,0.],[0.,0.,0.,9.0,0.,0.],[0.,0.,0.,0.,9.5,0.],[0.,0.,0.,0.,0.,8.0]))
Gtilde3 = np.array(([.1,0.,0.,0.,0.,0.],[0.,.1,0.,0.,0.,0.],[0.,0.,.01,0.,0.,0.],[0.,0.,0.,3.0,0.,0.],[0.,0.,0.,0.,1.5,0.],[0.,0.,0.,0.,0.,2.0]))

Gtildelist = np.array((Gtilde1,Gtilde2,Gtilde3))
Mtildelist = np.array((Mhat01,Mhat12,Mhat23))

Ftipmat = [[1,1,1,1,1,1]]*len(traj)

Kp = 20
Ki = 10
Kd = 18
intRes = 8

taumat,thetamat = r.SimulateControl(thetalist,dthetalist,g,Ftipmat,Mlist,Glist,Slist,thetamatd,dthetamatd,ddthetamatd,gtilde,Mtildelist,Gtildelist,Kp,Ki,Kd,dt,intRes)
    '''
    n = len(thetamatd)
    if Ftipmat == 0:
        NewFtipmat = [[0,0,0,0,0,0]]*n
    else:
        NewFtipmat = Ftipmat;
    thetacurrent = thetalist
    dthetacurrent = dthetalist
    eint = 0
    taumat = []
    thetamat = []
    taulist = ComputedTorque(thetacurrent,dthetacurrent,eint,gtilde,Mtildelist,Gtildelist,Slist,thetamatd[0],dthetamatd[0],ddthetamatd[0],Kp,Ki,Kd)
    for j in range (0,intRes):
        ddthetalist = ForwardDynamics(thetacurrent,dthetacurrent,taulist,g,NewFtipmat[0],Mlist,Glist,Slist)
        thetacurrent,dthetacurrent = EulerStep(thetacurrent,dthetacurrent,ddthetalist,(dt/intRes))
        taumat.append(taulist)
        thetamat.append(thetacurrent)
    eint = eint + matmult(dt,np.subtract(thetamatd[0], thetacurrent))

    for i in range (1,n):
       taulist = ComputedTorque(thetacurrent,dthetacurrent,eint,gtilde,Mtildelist,Gtildelist,Slist,thetamatd[i],dthetamatd[i],ddthetamatd[i],Kp,Ki,Kd)
       for j in range (0,intRes):
           ddthetalist = ForwardDynamics(thetacurrent,dthetacurrent,taulist,g,NewFtipmat[i],Mlist,Glist,Slist)
           thetacurrent,dthetacurrent = EulerStep(thetacurrent,dthetacurrent,ddthetalist,(dt/intRes))
           taumat.append(taulist)
           thetamat.append(thetacurrent)
       eint = eint + matmult(dt,np.subtract(thetamatd[i], thetacurrent))

    #Output using matplotlib to plot
    links  = len(thetamat[0])
    plotAngles =  [[None]*(len(thetamat)) for _ in range(links)]
    plotAnglesd =  [[None]*(len(thetamatd)) for _ in range(links)]

    for i in range(0,len(thetamat)):
        for j in range(0,links):
            plotAngles[j][i] = thetamat[i][j]

    for i in range(0,len(thetamatd)):
        for j in range(0,links):
            plotAnglesd[j][i] = thetamatd[i][j]

    N = len(thetamat)
    Tf = len(taumat)*dt
    timestamp = np.linspace(0,Tf,N)
    timestampd = np.linspace(0,Tf,(N/intRes))

    for i in range(0,links):
        col=[random.uniform(0,1),random.uniform(0,1),random.uniform(0,1)]
        plt.plot(timestamp, plotAngles[i],"-",color=col, label = ("ActualTheta"+str(i+1)))
        plt.plot(timestampd, plotAnglesd[i],".",color=col, label = ("DesiredTheta"+str(i+1)))

    plt.legend(loc = 'upper left')
    plt.xlabel("Time")
    plt.ylabel("Joint Angles")
    plt.title("Plot of Actual and Desired Joint Angles")
    plt.show()
        
    return (taumat,thetamat)
