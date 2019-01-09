function AdT = Adjoint(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes T a transformation matrix SE3. 
% Returns the corresponding 6x6 adjoint representation [AdT].
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% AdT = Adjoint(T)
% 
% Output:
% AdT =
%     1     0     0     0     0     0
%     0     0    -1     0     0     0
%     0     1     0     0     0     0
%     0     0     3     1     0     0
%     3     0     0     0     0    -1
%     0     0     0     0     1     0

[R, p] = TransToRp(T);
AdT = [R, zeros(3); VecToso3(p) * R, R];
end