%**********************************************************************************************
%*******************************  BASIC HELPER FUNCTIONS  *************************************
%**********************************************************************************************

function mag = Magnitude(V)
% Takes in a vector and returns its length
% Example Input:
%{
  clear;clc;
  V = [1,2,3];
  mag = Magnitude(V)
%}
% Output:
% mag =
%    3.7417

[m,n]=size(V);
if (n==1 && m~=0) || (m==1 && n~=0)
    mag=0;
    for i=1:length(V)
        mag=mag+V(i)^2;
    end
    mag=sqrt(mag);
elseif n~=0 && m~=0
    mag=zeros(1,n);
    for j=1:n
        for i=1:m
            mag(1,j)=mag(1,j)+V(i,j)^2;
        end
        mag(1,j)=sqrt(mag(1,j));
    end 
else
    msg = 'Input is not a vector.';
    error(msg);
end

