%**********************************************************************************************
%****************************  CHAPTER 9: TRAJECTORY GENERATION  ******************************
%**********************************************************************************************

function s = CubicTimeScaling(Tf,t)
% Takes Tf: Total time of the motion in seconds from rest to rest,
% t: The current time t satisfying 0<t<Tf.
%
% Returns s: The path parameter s(t) corresponding to a third-order polynomial motion that begins and
% ends at zero velocity
% Example Input: 
%{
  clear;clc;
  Tf = 2;
  t = 0.6;
  s = CubicTimeScaling(Tf,t)
%}
% Output:
% s =
%    0.2160
if t<=Tf && t>=0
    a2 = (3/Tf^2);
    a3 = (-2/Tf^3);
    s= (a2*(t^2))+(a3*(t^3));
else
    msg = 'Input is not appropriate.';
    error(msg);
end
end

