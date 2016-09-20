%**********************************************************************************************
%***************************  CHAPTER 8: DYNAMICS OF OPEN CHAINS  *****************************
%**********************************************************************************************

function taulist = InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist)
% Takes thetalist: n-vector of joint variables,
% dthetalist: n-vector of joint rates,
% ddthetalist: n-vector of joint accelerations,
% g: Gravity vector g,
% Ftip: Spatial force applied by the end-effector expressed in frame {n+1},
% Mlist: List of link frames {i} relative to {i-1} at the home position,
% Glist: Spatial inertia matrices Gi of the links,
% Slist: Screw axes Si of the joints in a space frame.
%
% Returns taulist: The n-vector of required joint forces/torques.
% This function uses forward-backward Newton-Euler iterations to solve the equation:
% taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) + g(thetalist) + Jtr(thetalist)Ftip
%
% Example Input (3 Link Robot):
%{ 
  clear;clc;
  thetalist = [0.1,0.1,0.1];
  dthetalist = [0.1,0.2,0.3];
  ddthetalist = [2,1.5,1];
 
  g = [0,0,-9.8];
  Ftip = [1,1,1,1,1,1];
 
  M01 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,0.,.089159,1.]]';
  M12 = [[0.,0.,-1.,0.];[0.,1.,0.,0.];[1.,0.,0.,0.];[.28,.13585,0.,1.]]';
  M23 = [[1.,0.,0.,0.];[0.,1.,0.,0.];[0.,0.,1.,0.];[0.,-.1197,.395,1]]';
 
  G1 = [[.010267,0.,0.,0.,0.,0.];[0.,.010267,0.,0.,0.,0.];[0.,0.,.00666,0.,0.,0.];[0.,0.,0.,3.7,0.,0.];[0.,0.,0.,0.,3.7,0.];[0.,0.,0.,0.,0.,3.7]];
  G2 = [[.22689,0.,0.,0.,0.,0.];[0.,.22689,0.,0.,0.,0.];[0.,0.,.0151074,0.,0.,0.];[0.,0.,0.,8.393,0.,0.];[0.,0.,0.,0.,8.393,0.];[0.,0.,0.,0.,0.,8.393]];
  G3 = [[.0494433,0.,0.,0.,0.,0.];[0.,.0494433,0.,0.,0.,0.];[0.,0.,.004095,0.,0.,0.];[0.,0.,0.,2.275,0.,0.];[0.,0.,0.,0.,2.275,0.];[0.,0.,0.,0.,0.,2.275]];
 
  Glist = [G1;G2;G3];
  Mlist = [M01;M12;M23];
 
  Slist = [[1.,0.,1.,0.,1.,0.];[0.,1.,0.,-.089,0.,0.];[0.,1.,0.,-.089,0.,.425]];
  taulist = InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist)
%}
% Output:
% taulist =
%   12.2874  -20.5555   -2.9404

%******INITIALISATION********
n=size(Slist,1);
Mi=Mlist(1:4,:);
Ai = (Adjoint(TransInv(Mi))*Slist(1,:)')';
Ti = Mi*MatrixExp6(Ai*thetalist(1));
Vi = ((Adjoint(TransInv(Ti)))*([0,0,0,0,0,0]'))'+(Ai*dthetalist(1));
Vdi = ((Adjoint(TransInv(Ti)))*([0,0,0,-g(1),-g(2),-g(3)]'))' + ((ad(Vi)*Ai')'*dthetalist(1)) + (Ai*ddthetalist(1));
%****************************
%*****Forward Iteration******
for i=2:n
    Mi = [Mi;Mi(size(Mi,1)-3:size(Mi,1),:)*Mlist(4*i-3:4*i,:)];
    Ai = [Ai;(Adjoint(TransInv(Mi(size(Mi,1)-3:size(Mi,1),:)))*Slist(i,:)')'];
    Ti = [Ti;Mlist(4*i-3:4*i,:)*MatrixExp6(Ai(i,:)*thetalist(i))];
    Vi = [Vi;((Adjoint(TransInv(Ti(4*i-3:4*i,:))))*(Vi(i-1,:)'))'+(Ai(i,:)*dthetalist(i))];
    Vdi = [Vdi;((Adjoint(TransInv(Ti(4*i-3:4*i,:))))*(Vdi(i-1,:)'))' + ((ad(Vi(i,:))*Ai(i,:)')'*dthetalist(i)) + (Ai(i,:)*ddthetalist(i))];
end
%****************************

%******INITIALISATION********
Fi=zeros(n,6);
Fi(n,:) = (Adjoint(TransInv(Ti(4*n-3:4*n,:))))'*Ftip' + (Glist(6*n-5:6*n,:)*Vdi(n,:)') - ((ad(Vi(n,:))')*(Vi(n,:)*Glist(6*n-5:6*n,:))');
taulist = zeros(1,n);
taulist(n) = ((Fi(n,:))*Ai(n,:)');
%****************************
%*****Backward Iteration*****
for i = n-1:-1:1
    Fi(i,:) = (Adjoint(TransInv(Ti(4*i-3:4*i,:))))'*Ftip' + (Glist(6*i-5:6*i,:)*Vdi(i,:)') - ((ad(Vi(i,:))')*(Vi(i,:)*Glist(6*i-5:6*i,:))');
    taulist(i) = ((Fi(i,:))*Ai(i,:)');
end
%****************************
