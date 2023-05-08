function  [q1,Fk]=Quat_Propag(w,q0,ST)
% propag   := discreate Propagation of angular velocity vector
%==========================================================================
% q1  (4*1):= quaternion vector at the end of sampling period
% w   (3*1):= angular velocity vector during the sampling period
% q0  (4*1):= quaternion vector at the begining of sampling period
% ST  (1*1):= Sample Time
%==========================================================================
W=skewSym4(w);
Z=0.5*W*ST;
z=sqrt(Z(2,1)^2+Z(3,1)^2+Z(4,1)^2);
if z==0
    z=eps;
end
expZ=cos(z)*eye(4)+sin(z)/(z)*Z;
%expZ=cos(z)*eye(4)+(1-z^2/6+z^4/120-z^7/5040)*Z;
q1=expZ*q0;
Fk=expZ;
%==========================================================================
end