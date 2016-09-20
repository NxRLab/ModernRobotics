%**********************************************************************************************
%*******************************  BASIC HELPER FUNCTIONS  *************************************
%**********************************************************************************************

function norm_v = Normalize(V)
% Takes in a vector and scales it to a unit vector
% Example Input:
%{
  clear;clc;
  V = [1,2,3];
  norm_v = Normalize(V)
%}
% Output:
% norm_v =
%    0.2673    0.5345    0.8018

[m,n]=size(V);
mag=Magnitude(V);
if ((n==1 && m~=0) || (m==1 && n~=0))&& mag~=0 && ~isempty(mag)
    norm_v=V./mag;
elseif (n~=0 && m~=0) && ~isempty(mag)
    norm_v=zeros(m,n);
    for i=1:length(mag)
        if mag(i)~=0
            norm_v(:,i)=V(:,i)/mag(i);
        end
    end
else
    msg = 'Input is not a vector.';
    error(msg);
end

end

