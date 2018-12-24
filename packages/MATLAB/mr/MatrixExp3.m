function  R = MatrixExp3(so3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 3x3 so(3) representation of exponential coordinates.
% Returns R in SO(3) that is achieved by rotating about omghat by theta 
% from an initial orientation R = I.
% Example Input:
% 
% clear; clc;
% so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
% R = MatrixExp3(so3mat)  
% 
% Output:
% R =
%   -0.6949    0.7135    0.0893
%   -0.1920   -0.3038    0.9332
%    0.6930    0.6313    0.3481

omgtheta = so3ToVec(so3mat);
if NearZero(norm(omgtheta))
    R = eye(3);
else
    [omghat, theta] = AxisAng3(omgtheta);
    omgmat = so3mat / theta;
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
end
end