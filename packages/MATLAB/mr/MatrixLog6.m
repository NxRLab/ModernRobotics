function expmat = MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T in SE(3).
% Returns the corresponding se(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% expmat = MatrixLog6(T)
% 
% Output:
% expc6 =
%         0         0         0         0
%         0         0   -1.5708    2.3562
%         0    1.5708         0    2.3562
%         0         0         0         0

[R, p] = TransToRp(T);
omgmat = MatrixLog3(R);
if isequal(omgmat, zeros(3))
    expmat = [zeros(3), T(1: 3, 4); 0, 0, 0, 0];
else
    theta = acos((trace(R) - 1) / 2);
    expmat = [omgmat, (eye(3) - omgmat / 2 ...
                      + (1 / theta - cot(theta / 2) / 2) ...
                        * omgmat * omgmat / theta) * p;
              0, 0, 0, 0];    
end
end