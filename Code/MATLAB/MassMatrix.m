%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function M = MassMatrix(thetalist,Mlist,Glist,Slist)
% Takes thetalist: A list of joint variables,
% Mlist: List of link frames i relative to i-1 at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame.
%
% Returns M: The numerical inertia matrix M(thetalist) of an n-joint serial chain at the
% given configuration thetalist.
% This function calls InverseDynamics n times, each time passing a ddthetalist vector
% with a single element equal to one and all other inputs set to zero. 
% Each call of InverseDynamics generates a single column,
% and these columns are assembled to create the inertia matrix.
%
% Example Input (3 Link Robot):
%{
  clear;clc;
  thetalist = [0.1,0.1,0.1];

  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';
 
  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];
 
  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];
 
  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  M = MassMatrix(thetalist,Mlist,Glist,Slist)
%}
% Output:
% M =
%    3.0866   -0.2860   -0.0072
%         0    0.8849    0.4322
%         0         0    0.1916
n=size(Slist,1);
M=[];
dthetalist=zeros(1,n);
g=[0;0;0];
Ftip=zeros(1,6);
for i=1:n
   ddthetalist=zeros(1,n);
   ddthetalist(i)=1;
   taulist=InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist);
   M=[M;taulist];
end
end

