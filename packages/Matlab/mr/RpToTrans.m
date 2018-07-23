function T = RpToTrans(R, p)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes rotation matrix R and position p.
% Returns the corresponding homogeneous transformation matrix T in SE(3).
% Example Input:
% 
% clear; clc;
% R = [[1, 0, 0]; [0, 0, -1]; [0, 1, 0]];
% p = [1; 2; 5];
% T = RpToTrans(R, p)
% 
% Output:
% T =
%     1     0     0     1
%     0     0    -1     2
%     0     1     0     5
%     0     0     0     1

T = [R, p; 0, 0, 0, 1];
end