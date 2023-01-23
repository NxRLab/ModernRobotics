import numpy as np
import modern_robotics as mr

def vlogR(R):
    tr = (np.trace(R) - 1) / 2
    tr = min(max(tr, -1), 1)
    fai = np.arccos(tr)
    if fai == 0:
        w = np.array([0, 0, 0])
    else:
        w = fai / (2 * np.sin(fai)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w


def Registration(X, Y, Rn=None, tn=None, Ln=None):
    if X.shape[0] != 3 or Y.shape[0] != 3:
        raise ValueError('Each argument must have exactly three rows.')
    elif X.shape[1] != Y.shape[1]:
        raise ValueError('X and Y must have the same number of columns.')
    elif X.shape[1] < 3 or Y.shape[1] < 3:
        raise ValueError('X and Y must each have 3 or more columns.')
    Npoints = X.shape[1]
    Xbar = np.mean(X, axis=1)
    Ybar = np.mean(Y, axis=1)
    Xtilde = X - np.tile(Xbar, (Npoints, 1)).T
    Ytilde = Y - np.tile(Ybar, (Npoints, 1)).T
    
    H = np.dot(Xtilde, Ytilde.T)
    
    U, S, V = np.linalg.svd(H)

    idx = np.argsort(S)[::-1]
    S = S[idx]
    U = U[:,idx]
    V = V[idx,:].T


    R = np.dot(np.dot(V, np.diag([1, 1, np.linalg.det(np.dot(V, U.T))])), U.T)
    t = Ybar - np.dot(R, Xbar)
    if Rn is not None and tn is not None and Ln is not None:
        ep = np.linalg.norm(t - tn)
        eo = np.linalg.norm(vlogR(R/Rn))
        et = ep + eo * Ln
        return R, t, ep, eo, et
    else:
        return R, t


def rotationW(g, theta):

    if theta==0:
        w=np.array([[0],[0],[0]])
        
    else:
        w=1/(2*np.sin(theta)) * np.array([
                                        [g[2,1]-g[1,2]],
                                        [g[0,2]-g[2,0]],
                                        [g[1,0]-g[0,1]] ])

    return w
    

def rotationTheta( g ):
    tr=(np.trace(g[0:3, 0:3]) - 1) / 2 
    if tr>=1:
        tr=1

    elif tr<=-1:

        tr=-1

    theta=np.arccos(tr)

    return theta


def vlog(g):

    fai=rotationTheta(g)
    w=fai*rotationW(g,fai)
    w = w.reshape((3,))

    if fai == 0:

        p=g[0:3,3]

    else:

        A = np.eye(3)-0.5*mr.VecToso3(w)
        b = (2*np.sin(fai)-fai*(1+np.cos(fai))) / (2*fai*fai*np.sin(fai))

        B = np.dot(np.dot(b,mr.VecToso3(w)),mr.VecToso3(w)) 

        p = np.dot((A+B),g[0:3,3])
    
    kesi=np.r_[w,p]

    return kesi


def aMatrix( kesi, q ):

    w=kesi[0:3]
    v=kesi[3:6]
    bW=np.zeros((6,6))
    bW[0:3,0:3]=mr.VecToso3(w)
    bW[3:6,0:3]=mr.VecToso3(v)
    bW[3:6,3:6]=mr.VecToso3(w)
    n=np.linalg.norm(w)
    t=n*q
    if n==0:
        aM=q*np.eye(6)
    else:
        a0 = q * np.eye(6)

        a10 = ( ( 4 - t * np.sin(t) - 4 * np.cos(t) ) / 2 / n**2 )
        a11 = np.dot(a10,bW) 

        a20 = ( ( 4 * t - 5 * np.sin(t) + t * np.cos(t) ) / 2 / n**3 )
        a21 = np.dot(np.dot(a20,bW),bW)

        a30 = ((2-t*np.sin(t)-2*np.cos(t))/2/n**4)
        a31 = np.dot(np.dot(np.dot(a30,bW),bW),bW)

        a40 = ((2*t-3*np.sin(t)+t*np.cos(t))/2/n**5)
        a41 = np.dot(np.dot(np.dot(np.dot(a40,bW),bW),bW),bW)

        aM = a0 + a11 + a21 + a31 + a41

    return aM

def aMatrixST( kesi ):

    w=kesi[0:3]
    v=kesi[3:6]
    bW=np.zeros((6,6))
    bW[0:3,0:3]=mr.VecToso3(w)
    bW[3:6,0:3]=mr.VecToso3(v)
    bW[3:6,3:6]=mr.VecToso3(w)
    t=np.linalg.norm(w)

    if t==0:

        aM = np.eye(6)

    else:

        a0 = np.eye(6)

        a10 = ( ( 4 - t * np.sin(t) - 4 * np.cos(t) ) / 2 / t**2 )
        a11 = np.dot(a10,bW) 

        a20 = ( ( 4 * t - 5 * np.sin(t) + t * np.cos(t) ) / 2 / t**3 )
        a21 = np.dot(np.dot(a20,bW),bW)

        a30 = ((2-t*np.sin(t)-2*np.cos(t))/2/t**4)
        a31 = np.dot(np.dot(np.dot(a30,bW),bW),bW)

        a40 = ((2*t-3*np.sin(t)+t*np.cos(t))/2/t**5)
        a41 = np.dot(np.dot(np.dot(np.dot(a40,bW),bW),bW),bW)

        aM = a0 + a11 + a21 + a31 + a41

    return aM


def dexp( kesi, theta ):

    J = np.c_[aMatrix(kesi,theta), kesi]

    return J


def se3Translation( v,theta ):

    T=np.r_[np.c_[np.eye(3),v*theta],np.array([0,0,0,1]).reshape((1,4))]

    return T


def rotationMatrix( w,theta ):

    R= np.eye(3) + mr.VecToso3(w) * np.sin(theta) + np.dot(mr.VecToso3(w), mr.VecToso3(w)) * (1-np.cos(theta))

    return R


def se3Rotation( w,v,theta ):

    R=rotationMatrix(w,theta)
    p = np.dot(theta*np.eye(3)+(1-np.cos(theta))*mr.VecToso3(w)+(theta-np.sin(theta))* np.dot(mr.VecToso3(w),mr.VecToso3(w)),v)
    T=np.r_[np.c_[R,p.reshape((3,1))], np.array([0,0,0,1]).reshape(1,4)]

    return T
        

def se3Exp( kesi ):

    n1=np.linalg.norm(kesi[0:3])
    n2=np.linalg.norm(kesi[3:6])

    if n1==0 and n2==0:
        T=np.eye(4)

    elif n1==0:
        T=se3Translation(kesi[3:6]/n2,n2)

    else:
        T=se3Rotation(kesi[0:3]/n1,kesi[3:6]/n1,n1)

    return T


def fKin(xi,theta,n):
    
    T = np.eye(4)
    for i in range(0,n):
        T = np.dot(T,se3Exp(np.dot(xi[:,i],theta[i])))
    
    return T


def traditionalCalibrationScara(xi0, vtheta, gm, M):

    xi = np.copy(xi0)

    M_home = np.eye(4)

    dq=np.zeros((4,1))
    N=np.size(vtheta,0)

    gn=np.zeros((4,4,N))
    dg=np.zeros((4,4,N))
    vLog=np.zeros((6,N))

    for i in range(0,N):

        gn[:,:,i]= mr.FKinBody(M_home, xi, vtheta[i,:])
        dg[:,:,i] = np.linalg.solve(gn[:,:,i].T, gm[:,:,i].T).T
        vLog[:,i]= vlog(dg[:,:,i])

    error=np.zeros((3,1))

    for i in range(0,N):

        error=error+ np.array([
            [np.linalg.norm(vLog[:,i])],
            [np.linalg.norm(vLog[3:6,i])],
            [np.linalg.norm(vLog[0:3,i])]])


    simJ = np.zeros((60,25))
    meanE = np.zeros((3,11))

    meanE[:,0]=(error/N).reshape((3,))

    convergence=np.zeros((10,2))

    for m in range(0,10):

        for i in range(0,N):

            gn[:,:,i]= mr.FKinBody(M_home, xi, vtheta[i,:])
            dg[:,:,i]= np.linalg.solve(gn[:,:,i].T,gm[:,:,i].T).T 
            vLog[:,i] = vlog(dg[:,:,i])

        simY=np.zeros((6*N,1))

        for i in range(0,N):

            simY[6*(i+1)-6 : 6 * (i+1), 0 ] = vLog[:,i]
        
        for k in range(0,N-1):

            simJ[0+6*k:6+6*k,0:7] = dexp(xi[:,0], vtheta[k,0])

            simJ[0+6*k:6+6*k,7:14]= np.dot( mr.Adjoint( se3Exp( np.dot(vtheta[k,0], xi[:,0])) ), dexp( xi[:,1], vtheta[k,1] ))

            tempSIMJ= np.dot(mr.Adjoint( np.dot(se3Exp( np.dot(vtheta[k,0], xi[:,0])), se3Exp( np.dot(vtheta[k,1], xi[:,1])))), dexp( xi[:,2], vtheta[k,2]))

            simJ[6*k:6+6*k,14:18] = tempSIMJ[:,3:7]

            simJ[0+6*k:6+6*k,18:25] = np.dot(mr.Adjoint( np.dot(np.dot(se3Exp( np.dot(vtheta[k,0], xi[:,0]) ), se3Exp( np.dot( vtheta[k,1], xi[:,1] ) )), se3Exp( np.dot( vtheta[k,2], xi[:,2] ) )) ),dexp(xi[:,3],vtheta[k,3]))

            dp = np.linalg.lstsq(simJ, simY, rcond=None)[0]


        xi[:,0] = (xi[:,0].reshape((6,1))+dp[0:6].reshape((6,1))).reshape((6,))
        xi[0:3,0] = xi[0:3,0] / np.linalg.norm(xi[0:3,0])
        xi[3:6,0] = xi[3:6,0] + np.dot( np.dot(xi[0:3,1].T, xi[3:6,0]) /  np.dot(xi[0:3,0].T, xi[0:3,0] ), xi[0:3,0])

        xi[:,1] = (xi[:,1].reshape((6,1)) + dp[7:13].reshape((6,1))).reshape((6,))
        xi[0:3,1] = xi[0:3,1]/np.linalg.norm(xi[0:3,1])
        xi[3:6,1] = xi[3:6,1] - np.dot( np.dot(xi[0:3,1].T, xi[3:6,1]) / np.dot(xi[0:3,1].T,xi[0:3,1]), xi[0:3,1])

        xi[3:6,2]=(xi[3:6,2].reshape((3,1)) + dp[14:17].reshape((3,1))).reshape((3,))
        xi[3:6,2]=xi[3:6,2] / np.linalg.norm(xi[3:6,2])

        xi[:,3]= (xi[:,3].reshape((6,1)) + dp[18:24].reshape((6,1))).reshape((6,))
        xi[0:3,3]=xi[0:3,3] / np.linalg.norm(xi[0:3,3])
        xi[3:6,3]= xi[3:6,3] - np.dot(np.dot(xi[0:3,3].T, xi[3:6,3]) / np.dot(xi[0:3,3].T,xi[0:3,3]),xi[0:3,3])

        vtheta[:,0] = vtheta[:,0]+dp[6]
        vtheta[:,1] = vtheta[:,1]+dp[13]
        vtheta[:,2] = vtheta[:,2]+dp[17]
        vtheta[:,3] = vtheta[:,3]+dp[24]

        dq = dq + np.array([dp[6], dp[13], dp[17], dp[24]])

        for i in range(0,N):
            gn[:,:,i] = mr.FKinBody(M_home, xi, vtheta[i,:])
            dg[:,:,i] = np.linalg.solve(gn[:,:,i].T,gm[:,:,i].T).T
            vLog[:,i] = vlog(dg[:,:,i])

        error=np.zeros((3,1))

        for i in range(0,N):
            
            error=error + np.r_[ np.r_[ np.linalg.norm(vLog[:,i]), np.linalg.norm(vLog[3:6,i]) ], np.linalg.norm(vLog[0:3,i]) ].reshape((3,1))

        meanE[:, m] = (error/N).reshape((3,))
        convergence[m,0]=np.linalg.norm(simY)
        convergence[m,1]=np.linalg.norm(dp)

    return xi, dq, meanE, convergence


def main(argv, argc):
    pass

if __name__ == "__main__":
    main()