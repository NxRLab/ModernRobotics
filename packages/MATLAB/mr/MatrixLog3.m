function so3mat = MatrixLog3(R)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes R (rotation matrix).
% Returns the corresponding so(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% R = [[0, 0, 1]; [1, 0, 0]; [0, 1, 0]];
% so3mat = MatrixLog3(R)
% 
% Output:
% angvmat =
%         0   -1.2092    1.2092
%    1.2092         0   -1.2092
%   -1.2092    1.2092         0

acosinput = (trace(R) - 1) / 2;
if acosinput >= 1
    so3mat = zeros(3);
elseif acosinput <= -1
    if ~NearZero(1 + R(3, 3))
        omg = (1 / sqrt(2 * (1 + R(3, 3)))) ...
              * [R(1, 3); R(2, 3); 1 + R(3, 3)];
    elseif ~NearZero(1 + R(2, 2))
        omg = (1 / sqrt(2 * (1 + R(2, 2)))) ...
              * [R(1, 2); 1 + R(2, 2); R(3, 2)];
    else
        omg = (1 / sqrt(2 * (1 + R(1, 1)))) ...
              * [1 + R(1, 1); R(2, 1); R(3, 1)];
    end
    so3mat = VecToso3(pi * omg);
else
	theta = acos(acosinput);
    so3mat = theta * (1 / (2 * sin(theta))) * (R - R');
end
end