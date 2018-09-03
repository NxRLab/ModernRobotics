function norm_v = Normalize(V)
% *** BASIC HELPER FUNCTIONS ***
% Takes in a vector.
% Scales it to a unit vector.
% Example Input:
% 
% clear; clc;
% V = [1; 2; 3];
% norm_v = Normalize(V)
% 
% Output:
% norm_v =
%    0.2673
%    0.5345
%    0.8018

norm_v = V / norm(V);
end

