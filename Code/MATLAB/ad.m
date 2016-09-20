%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function adV = ad(V)
% Takes V: 6-vector spatial velocity 
% Returns adV: The corresponding 6 × 6 matrix
% Used to calculate the Lie bracket [V1, V2] = [adV1]V2
% Example Input:
%{ 
  clear;clc;
  V = [1,2,3,4,5,6];
  adV = ad(V)
%}
% 
% Output:
% adV =
%     0    -3     2     0     0     0
%     3     0    -1     0     0     0
%    -2     1     0     0     0     0
%     0    -6     5     0    -3     2
%     6     0    -4     3     0    -1
%    -5     4     0    -2     1     0
w=V(1:3);
v=V(4:6);
adV=[VecToso3(w),zeros(3);VecToso3(v),VecToso3(w)];
end

