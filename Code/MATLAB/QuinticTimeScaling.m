%**********************************************************************************************
%****************************  CHAPTER 9: TRAJECTORY GENERATION  ******************************
%**********************************************************************************************

function s = QuinticTimeScaling(Tf,t)
% Takes Tf: Total time of the motion in seconds from rest to rest,
% t: The current time t satisfying 0<t<Tf.
%
% Returns s: The path parameter s(t) corresponding to a fifth-order polynomial motion that begins and
% ends at zero velocity and zero acceleration.
% Example Input: 
%{
  clear;clc;
  Tf = 2;
  t = 0.6;
 s = QuinticTimeScaling(Tf,t)
%}
% Output:
% s =
%    0.1631
if t<=Tf && t>=0
    a3 = (10/(Tf^3));
    a4 = (-15/(Tf^4));
    a5 = (6/(Tf^5));
    s= ((a3*(t^3))+(a4*(t^4))+(a5*(t^5)));
else
    msg = 'Input is not appropriate.';
    error(msg);
end
end
