function invR = RotInv(R)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 3x3 rotation matrix.
% Returns the inverse (transpose).
% Example Input:
% 
% clear; clc;
% R = [0, 0, 1; 1, 0, 0; 0, 1, 0];
% invR = RotInv(R)
% 
% Output:
% invR =
%     0     1     0
%     0     0     1
%     1     0     0

invR = transpose(R);
end
