function invT = TransInv(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T.
% Returns its inverse. 
% Uses the structure of transformation matrices to avoid taking a matrix
% inverse, for efficiency.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% invT = TransInv(T)
% 
% Ouput:
% invT =
%     1     0     0     0
%     0     0     1    -3
%     0    -1     0     0
%     0     0     0     1

[R, p] = TransToRp(T);
invT = [transpose(R), -transpose(R) * p; 0, 0, 0, 1];
end
