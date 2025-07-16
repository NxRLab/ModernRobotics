import torch
'''
***************************************************************************
Modern Robotics: Mechanics, Planning, and Control.
Code Library for PyTorch
***************************************************************************
Author: Ayush Gaggar
        Adapted from original Python code by Huan Weng, Bill Hunt, Jarvis Schultz, Mikhail Todes, Kevin Lynch, and Frank Park
Email: agaggar@u.northwestern.edu
Date: July 2025
***************************************************************************
Language: Python
Required library: pytorch
***************************************************************************

This code is a PyTorch implementation of the mathematical operations and transformations in the Modern Robotics package,
and so is best used for *batched* tensors, i.e., tensors with shape (N, ...), where N is the batch size.
'''

def NearZero(val: torch.Tensor, eps=1e-6):
    """Check if input tensor values are close to zero."""
    return torch.abs(val) < eps

def Normalize(v: torch.Tensor):
    """Normalizes a vector to unit length.

    :param v: A vector, shape (N, 3, 1), (N, 3), (3,)
    :return: A normalized vector, shape (N, 3)
    """
    if v.ndim == 1:
        v = v.unsqueeze(0)
    if v.ndim == 2:
        v = v.unsqueeze(-1)
    norm = torch.linalg.norm(v, dim=-2, keepdim=True)
    return v / norm.clamp(min=1e-6)  # Avoid division by zero

def RotInv(R: torch.Tensor):
    """Inverts a rotation matrix.

    :param R: A rotation matrix, shape (N, 3, 3)
    :return: The inverse of R, which is the transpose of R, shape (N, 3, 3)
    """
    if R.ndim == 2:
        R = R.unsqueeze(0)
    return torch.transpose(R, -2, -1)

def VecToso3(omega: torch.Tensor):
    """Converts a 3-vector to an so(3) representation

    :param omg: angular velocity vector, shape (N, 3)
    :return: The skew symmetric representation of omg, i.e., an so(3) matrix,
             shape (N, 3, 3)
    """
    if omega.ndim > 2:
        omega = omega.reshape(-1, 3)
    elif omega.ndim == 1:
        omega = omega.unsqueeze(0)
    wx, wy, wz = omega[:, 0], omega[:, 1], omega[:, 2]
    row0 = torch.stack([torch.zeros_like(wx), -wz, wy], dim=1)
    row1 = torch.stack([wz, torch.zeros_like(wy), -wx], dim=1)
    row2 = torch.stack([-wy, wx, torch.zeros_like(wz)], dim=1)
    so3 = torch.stack([row0, row1, row2], dim=1)
    return so3

def so3ToVec(so3mat: torch.Tensor):
    """Converts an so(3) representation to a 3-vector

    :param so3mat: A skew-symmetric matrix, shape (N, 3, 3)
    :return: The 3-vector corresponding to so3mat, shape (N, 3, 1)
    """
    return torch.stack([
        so3mat[:, 2, 1],
        so3mat[:, 0, 2],
        so3mat[:, 1, 0]
    ], dim=-1).unsqueeze(-1)


def AxisAng3(expc3: torch.Tensor):
    """Converts a 3-vector of exponential coordinates for rotation into
    axis-angle form

    :param expc3: A 3-vector of exponential coordinates for rotation, shape (N, 3, 1)
    :return omghat: A unit rotation axis, shape (N, 3)
    :return theta: The corresponding rotation angle, shape (N,)
    """
    if expc3.ndim == 2:
        expc3 = expc3.unsqueeze(-1)
    theta = torch.linalg.norm(expc3, dim=-2).reshape(expc3.shape[0])
    omega_hat = torch.zeros_like(expc3)
    nonzero = ~NearZero(theta)
    omega_hat[nonzero] = expc3[nonzero] / theta[nonzero][:, None, None]
    return omega_hat, theta

def MatrixExp3(so3mat: torch.Tensor):
    """Computes the matrix exponential of a matrix in so(3)

    :param so3mat: A skew-symmetric matrix, shape (N, 3, 3)
    :return: The matrix exponential of so3mat, shape (N, 3, 3)

    Example Input:
        so3mat = np.array([[ 0, -3,  2],
                           [ 3,  0, -1],
                           [-2,  1,  0]])
    Output:
        np.array([[-0.69492056,  0.71352099,  0.08929286],
                  [-0.19200697, -0.30378504,  0.93319235],
                  [ 0.69297817,  0.6313497 ,  0.34810748]])
    """
    omgtheta = so3ToVec(so3mat)
    nonzero = ~NearZero(torch.linalg.norm(omgtheta, dim=-2).reshape(so3mat.shape[0]))
    add_me = torch.zeros_like(so3mat)
    theta_nonzero = AxisAng3(omgtheta)[1][nonzero][:, None, None]
    omgmat_nonzero = so3mat[nonzero] / theta_nonzero
    add_me[nonzero] = torch.sin(theta_nonzero) * omgmat_nonzero + (1 - torch.cos(theta_nonzero)) * torch.bmm(omgmat_nonzero, omgmat_nonzero)
    identity = torch.eye(3, device=so3mat.device, dtype=so3mat.dtype)
    identity = identity.repeat(so3mat.shape[0], 1, 1)
    return identity + add_me

def MatrixLog3(R: torch.Tensor):
    """Computes the matrix logarithm of rotation matrix

    :param R: Nx3x3 rotation matrix, shape (N, 3, 3)
    :return: The matrix logarithm of R, shape (N, 3, 3)
    """
    #TODO: broadcast instead of iterating
    trace_R = R.diagonal(offset=0, dim1=1, dim2=2).sum(-1)  # (N,)
    acosinput = torch.clamp((trace_R - 1) / 2.0, -1.0, 1.0)
    logm = torch.zeros_like(R, device=R.device, dtype=R.dtype)
    for idx, rot in enumerate(R):
        if torch.abs(1 - acosinput[idx]) <= 1e-6:
            continue
        elif torch.abs(acosinput[idx] + 1) <= 1e-6:
            if not NearZero(1 + rot[2][2]):
                omg = (1.0 / torch.sqrt(torch.clip(2 * (1 + rot[2][2]), min=1e-6))) \
                      * torch.stack([rot[0][2], rot[1][2], 1 + rot[2][2]], dim=0)
            elif not NearZero(1 + rot[1][1]):
                omg = (1.0 / torch.sqrt(torch.clip(2 * (1 + rot[1][1]), min=1e-6))) \
                      * torch.stack([rot[0][1], 1 + rot[1][1], rot[2][1]], dim=0)
            else:
                omg = (1.0 / torch.sqrt(torch.clip(2 * (1 + rot[0][0]), min=1e-6))) \
                      * torch.stack([1 + rot[0][0], rot[1][0], rot[2][0]], dim=0)
            logm[idx] = VecToso3(torch.pi * omg)
        else:
            theta = torch.acos(acosinput[idx])
            logm[idx] = theta / (2.0 * torch.sin(theta)) * (rot - torch.transpose(rot, -2, -1))
    if torch.any(torch.isnan(logm)):
        raise ValueError("NaN values in MatrixLog3")
    return logm

def RpToTrans(R: torch.Tensor, p: torch.Tensor):
    """Converts rotation matrices and position vectors into homogeneous
    transformation matrices

    :param R: Rotation matrix, (N, 3, 3)
    :param p: A 3-vector, (N, 3)
    :return: A homogeneous transformation matrix corresponding to the inputs
    """
    if p.ndim == 2:
        p = p.unsqueeze(-1)
    bottom_row = torch.tensor([0, 0, 0, 1], device=R.device, dtype=R.dtype)
    bottom_row = bottom_row.expand(R.size(0), 1, 4)
    return torch.cat([torch.cat([R, p], dim=-1), 
                      bottom_row], dim=-2).float().to(device=R.device)

def TransToRp(T: torch.Tensor):
    """Converts a homogeneous transformation matrix into a rotation matrix
    and position vector, given T is (N, 4, 4).

    :param T: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix, (N, 3, 3)
    :return p: The corresponding position vector, (N, 3, 1)
    """
    if T.ndim == 2:
        T = T.unsqueeze(0)
    return T[:, 0: 3, 0: 3], T[:, 0: 3, 3].unsqueeze(-1)

def TransInv(T: torch.Tensor):
    """Inverts a homogeneous transformation matrix, given T is (N, 4, 4).

    :param T: A homogeneous transformation matrix, (N, 4, 4)
    :return: The inverse of T, (N, 4, 4)
    Uses the structure of transformation matrices to avoid taking a matrix
    inverse, for efficiency.
    """
    R, p = TransToRp(T)
    Rt = torch.transpose(R, -2, -1)
    if p.ndim == 2:
        p = p.unsqueeze(-1)
    bottom_row = torch.tensor([0, 0, 0, 1], device=R.device, dtype=R.dtype)
    bottom_row = bottom_row.expand(R.size(0), 1, 4)
    return torch.cat([torch.cat([Rt, -torch.bmm(Rt, p)], dim=-1), 
                      bottom_row], dim=-2)
    
def VecTose3(V: torch.Tensor):
    """Converts a spatial velocity vector into a 4x4 matrix in se3

    :param V: A 6-vector representing a spatial velocity, shape (N, 6)
    :return: The 4x4 se3 representation of V, shape (N, 4, 4)
    """
    omega = V[:, :3]
    v = V[:, 3:]
    if v.ndim == 2:
        v = v.unsqueeze(-1)
        omega = omega.unsqueeze(-1)
    # bottom_row = torch.zeros((V.shape[0], 1, 4), device=V.device, dtype=V.dtype)
    # bottom_row[:, 0, 3] = 1.
    bottom_row = torch.tensor([0, 0, 0, 1], device=V.device, dtype=V.dtype)
    bottom_row = bottom_row.expand(V.size(0), 1, 4)  # (N, 1, 4)
    return torch.cat([torch.cat([VecToso3(omega), v], dim=-1), 
                      bottom_row], dim=-2)

def se3ToVec(se3mat: torch.Tensor):
    """ Converts se3 matrices into spatial velocity vector

    :param se3mat: SE3 Matrix, shape (N, 4, 4)
    :return: The spatial velocity 6-vector corresponding to se3mat, shape (N, 6, 1)
    """
    if se3mat.ndim == 2:
        se3mat = se3mat.unsqueeze(0)
    vecm = torch.stack([
        se3mat[:, 2, 1],
        se3mat[:, 0, 2],
        se3mat[:, 1, 0],
        se3mat[:, 0, 3],
        se3mat[:, 1, 3],
        se3mat[:, 2, 3],
    ], dim=1)
    return vecm.unsqueeze(-1)

def Adjoint(T: torch.Tensor):
    """Computes the adjoint representation of a homogeneous transformation
    matrix

    :param T: A homogeneous transformation matrix, shape (N, 4, 4)
    :return: The adjoint representation of T, shape (N, 6, 6)
    """
    R, p = TransToRp(T)
    top_left = R
    top_right = torch.zeros((R.shape[0], 3, 3), device=T.device, dtype=T.dtype)
    bottom_left = torch.bmm(VecToso3(p), R)
    bottom_right = R
    return torch.cat([torch.cat([top_left, top_right], dim=-1),
                      torch.cat([bottom_left, bottom_right], dim=-1)], dim=-2)

def ScrewToAxis(q: torch.Tensor, s: torch.Tensor, h: torch.Tensor):
    """Takes a parametric description of a screw axis and converts it to a
    normalized screw axis.

    :param q: A point on the screw axis, shape (N, 3)
    :param s: The direction of the screw axis, shape (N, 3)
    :param h: The pitch of the screw, shape (N,)
    :return: The axis representation of the screw, shape (N, 6)
    """
    return NotImplementedError("ScrewToAxis is not yet tested.")
    if q.ndim == 2:
        q = q.unsqueeze(-1)
    if s.ndim == 2:
        s = s.unsqueeze(-1)
    if h.ndim == 1:
        h = h.unsqueeze(-1)
    
    return torch.cat([s, torch.cross(q.squeeze(-1), s.squeeze(-1), dim=-1) + h * s.squeeze(-1)], dim=-1).unsqueeze(-1)

def AxisAng6(expc6: torch.Tensor):
    """Converts a 6-vector of exponential coordinates into screw axis-angle
    form

    :param expc6: A 6-vector of exponential coordinates for rigid-body motion
                  S*theta, shape (N, 6) or (N, 6, 1)
    :return S: The corresponding normalized screw axis, shape (N, 6)
    :return theta: The distance traveled along/about S, shape (N, 1) or (N,)
    """
    return NotImplementedError("AxisAng6 is not yet tested.")
    theta = torch.linalg.norm(expc6[:, :3], dim=-1, keepdim=True)
    mask = NearZero(theta)
    theta[mask] = torch.linalg.norm(expc6[:, 3:], dim=-1, keepdim=True)
    return (expc6 / theta).squeeze(-1), theta.squeeze(-1)

def MatrixExp6(se3mat: torch.Tensor):
    """Computes the matrix exponential of an se3 representation of
    exponential coordinates

    :param se3mat: Matrix in se3, shape (N, 4, 4)
    :return: Matrix exponential of se3mat, shape (N, 4, 4)
    """
    omgtheta = so3ToVec(se3mat[:, :3, :3])
    nonzero = ~NearZero(torch.linalg.norm(omgtheta, dim=-2).reshape(se3mat.shape[0]))
    
    first_part = MatrixExp3(se3mat[:, :3, :3])
    second_part = se3mat[:, :3, 3].unsqueeze(-1).clone()
    identity = torch.eye(3, device=se3mat.device, dtype=se3mat.dtype)
    identity = identity.repeat(se3mat.shape[0], 1, 1)
    bottom_row = torch.tensor([0, 0, 0, 1], device=se3mat.device, dtype=se3mat.dtype)
    bottom_row = bottom_row.expand(se3mat.size(0), 1, 4)  # (N, 1, 4)

    theta_nonzero = AxisAng3(omgtheta)[1][nonzero][:, None, None]
    omgmat_nonzero = se3mat[:, :3, :3][nonzero] / theta_nonzero
    second_part[nonzero] = torch.bmm(identity[nonzero] * theta_nonzero \
                                        + (1 - torch.cos(theta_nonzero)) * omgmat_nonzero \
                                        + (theta_nonzero - torch.sin(theta_nonzero)) \
                                        * torch.bmm(omgmat_nonzero, omgmat_nonzero),
                                     second_part[nonzero] / theta_nonzero)
    omgmat_nonzero = se3mat[:, :3, :3][nonzero] / theta_nonzero
    
    return torch.cat([torch.cat([first_part, second_part], dim=-1), 
                      bottom_row], dim=-2)

def MatrixLog6(T: torch.Tensor):
    """Computes the matrix logarithm of a homogeneous transformation matrix

    :param T: Matrix in SE3, shape (N, 4, 4)
    :return: Matrix logarithm of T, shape (N, 4, 4)
    """
    rot, second = TransToRp(T)
    all_rots = MatrixLog3(rot)
    if second.ndim == 2:
        second = second.unsqueeze(-1)

    first = torch.zeros_like(rot, device=T.device, dtype=T.dtype)
    bottom_row = torch.zeros((T.shape[0], 1, 4), device=T.device, dtype=T.dtype)
    logm = torch.zeros_like(T, device=T.device, dtype=T.dtype)

    #TODO: broadcast instead of iterating
    nonzero = ~(NearZero(all_rots, eps=1e-4).sum(dim=2).sum(dim=1) == 9)
    logm[~nonzero] = torch.cat([torch.cat([first[~nonzero], second[~nonzero]], dim=-1),
                                 bottom_row[~nonzero]], dim=-2)
    trace_R = rot.diagonal(offset=0, dim1=1, dim2=2).sum(-1)  # (N,)
    if torch.any(torch.isnan(trace_R)):
        raise ValueError("NaN values in trace_R")
    theta_nonzero = torch.acos(torch.clip((trace_R - 1) / 2.0, -1., 1.))[nonzero]

    identity = torch.eye(3, device=T.device, dtype=T.dtype)
    identity = identity.repeat(T.shape[0], 1, 1)
    logm[nonzero] = torch.cat([torch.cat([all_rots[nonzero], 
                                          (identity[nonzero] - all_rots[nonzero] / 2.0 \
                                                + torch.clamp((1.0 / theta_nonzero - 1.0 / torch.tan(theta_nonzero / 2.0) / 2.), min=-1e3, max=1e4)[:, None, None] \
                                                * torch.bmm(all_rots[nonzero], all_rots[nonzero]) / theta_nonzero[:, None, None]) @ second[nonzero]], dim=-1),
                                 bottom_row[nonzero]], dim=-2)
    if torch.any(torch.isnan(logm)):
        raise ValueError("NaN values in MatrixLog6")
    return logm

def ProjectToSO3(R: torch.Tensor):
    """Projects a rotation matrix onto the nearest SO(3) matrix

    :param R: A rotation matrix, shape (N, 3, 3)
    :return: The projected rotation matrix, shape (N, 3, 3)
    """
    return NotImplementedError("ProjectToSO3 is not yet implemented.")

def ProjectToSE3(T: torch.Tensor):
    return NotImplementedError("ProjectToSE3 is not yet implemented.")

def DistanceToSO3(R: torch.Tensor):
    """Returns the Frobenius norm to describe the distance of mat from the
    SO(3) manifold

    :param R: A matrix, shape (N, 3, 3)
    :return: A quantity describing the distance of mat from the SO(3) manifold, shape (N,)
    """
    return NotImplementedError("DistanceToSO3 is not yet implemented.")

def DistanceToSE3(T: torch.Tensor):
    """Returns the Frobenius norm to describe the distance of mat from the
    SE(3) manifold

    :param T: A matrix, shape (N, 4, 4)
    :return: A quantity describing the distance of mat from the SE(3) manifold, shape (N,)
    """
    return NotImplementedError("DistanceToSE3 is not yet implemented.")

def TestIfSO3(R: torch.Tensor):
    return NotImplementedError("TestIfSO3 is not yet implemented.")

def TestIfSE3(T: torch.Tensor):
    return NotImplementedError("TestIfSE3 is not yet implemented.")

def FKinBody(M: torch.Tensor, Blist: torch.Tensor, thetalist: torch.Tensor):
    return NotImplementedError("FKinBody is not yet implemented.")

def FKinSpace(M: torch.Tensor, Slist: torch.Tensor, thetalist: torch.Tensor):
    return NotImplementedError("FKinSpace is not yet implemented.")

def JacobianBody(Blist: torch.Tensor, thetalist: torch.Tensor):
    return NotImplementedError("JacobianBody is not yet implemented.")

def JacobianSpace(Slist: torch.Tensor, thetalist: torch.Tensor):
    return NotImplementedError("JacobianSpace is not yet implemented.")

def IKinBody(Blist: torch.Tensor, M: torch.Tensor, T: torch.Tensor, thetalist0: torch.Tensor, eomg=1e-6, ev=1e-6):
    return NotImplementedError("IKinBody is not yet implemented.")

def IKinSpace(Slist: torch.Tensor, M: torch.Tensor, T: torch.Tensor, thetalist0: torch.Tensor, eomg=1e-6, ev=1e-6):
    return NotImplementedError("IKinSpace is not yet implemented.")

def ad(V: torch.Tensor):
    """Computes the adjoint representation of a spatial velocity vector

    :param V: A spatial velocity vector, shape (N, 6)
    :return: The adjoint representation of V, shape (N, 6, 6)
    """
    return NotImplementedError("ad is not yet tested.")
    if V.ndim == 1:
        V = V.unsqueeze(0)
    if V.ndim == 2:
        V = V.unsqueeze(-1)
    omg = V[:, :3]
    v = V[:, 3:]
    so3mat = VecToso3(omg)
    return torch.cat([torch.cat([so3mat, v], dim=-1),
                      torch.cat([VecToso3(v), so3mat], dim=-1)], dim=-2)

def InverseDynamics(thetalist: torch.Tensor, dthetalist: torch.Tensor, ddthetalist: torch.Tensor, g: torch.Tensor, Ftip: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("InverseDynamics is not yet implemented.")

def MassMatrix(thetalist: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("MassMatrix is not yet implemented.")

def VelQuadraticForces(thetalist: torch.Tensor, dthetalist: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("VelQuadraticForces is not yet implemented.")

def GravityForces(thetalist: torch.Tensor, g: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("GravityForces is not yet implemented.")

def EndEffectorForces(thetalist: torch.Tensor, Ftip: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("EndEffectorForces is not yet implemented.")

def ForwardDynamics(thetalist: torch.Tensor, dthetalist: torch.Tensor, taulist: torch.Tensor, g: torch.Tensor, Ftip: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("ForwardDynamics is not yet implemented.")

def EulerStep(thetalist: torch.Tensor, dthetalist: torch.Tensor, ddthetalist: torch.Tensor, dt: float):
    """Performs a single Euler step for the given joint angles, velocities, and accelerations.

    :param thetalist: Joint angles, shape (N, 6)
    :param dthetalist: Joint velocities, shape (N, 6)
    :param ddthetalist: Joint accelerations, shape (N, 6)
    :param dt: Time step for the Euler integration
    :return: Updated joint angles and velocities after the Euler step
    """
    return thetalist + dthetalist * dt, dthetalist + ddthetalist * dt

def InverseDynamicsTrajectory(thetamat: torch.Tensor, dthetamatrix: torch.Tensor, ddthetamatrix: torch.Tensor, g: torch.Tensor, Ftipmatrix: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor):
    return NotImplementedError("InverseDynamicsTrajectory is not yet implemented.")

def ForwardDynamicsTrajectory(thetamat: torch.Tensor, dthetamatrix: torch.Tensor, taumat: torch.Tensor, g: torch.Tensor, Ftipmatrix: torch.Tensor, Mlist: torch.Tensor, Glist: torch.Tensor, Slist: torch.Tensor, dt: float, intRes: int):
    return NotImplementedError("ForwardDynamicsTrajectory is not yet implemented.")

def CubicTimeScaling(Tf, t):
    """Computes s(t) for a cubic time scaling

    :param Tf: Total time of the motion in seconds from rest to rest
    :param t: The current time t satisfying 0 < t < Tf
    :return: The path parameter s(t) corresponding to a third-order
             polynomial motion that begins and ends at zero velocity

    Example Input:
        Tf = 2
        t = 0.6
    Output:
        0.216
    """
    return 3 * (1.0 * t / Tf) ** 2 - 2 * (1.0 * t / Tf) ** 3

def QuinticTimeScaling(Tf, t):
    """Computes s(t) for a quintic time scaling

    :param Tf: Total time of the motion in seconds from rest to rest
    :param t: The current time t satisfying 0 < t < Tf
    :return: The path parameter s(t) corresponding to a fifth-order
             polynomial motion that begins and ends at zero velocity and zero
             acceleration

    Example Input:
        Tf = 2
        t = 0.6
    Output:
        0.16308
    """
    return 10 * (1.0 * t / Tf) ** 3 - 15 * (1.0 * t / Tf) ** 4 \
           + 6 * (1.0 * t / Tf) ** 5

def JointTrajectory(thetalist: torch.Tensor, thetaend: torch.Tensor, Tf: float, N: int, method):
    return NotImplementedError("JointTrajectory is not yet implemented.")

def ScrewTrajectory(Xstart: torch.Tensor, Xend: torch.Tensor, Tf: float, N: int, method):
    return NotImplementedError("ScrewTrajectory is not yet implemented.")

def CartesianTrajectory(Xstart: torch.Tensor, Xend: torch.Tensor, Tf: float, N: int, method):
    return NotImplementedError("CartesianTrajectory is not yet implemented.")

def ComputedTorque(**kwargs):
    return NotImplementedError("ComputedTorque is not yet implemented.")

def SimulateControl(**kwargs):
    return NotImplementedError("SimulateControl is not yet implemented.")
