function T=Q2TM(q)
% Q2TM   := convert Quaternion to Transformation Matrix
%==========================================================================
% T (3*3):= Transformation matrix constructed from the quaternion vector
% q (4*1):= quaternion vector
%==========================================================================
T=zeros(3,3);
T(1,1)=q(1)^2+q(2)^2-q(3)^2-q(4)^2;
T(2,1)=2*(q(2)*q(3)-q(1)*q(4));
T(3,1)=2*(q(2)*q(4)+q(1)*q(3));
T(1,2)=2*(q(2)*q(3)+q(1)*q(4));
T(2,2)=q(1)^2-q(2)^2+q(3)^2-q(4)^2;
T(3,2)=2*(q(3)*q(4)-q(1)*q(2));
T(1,3)=2*(q(2)*q(4)-q(1)*q(3));
T(2,3)=2*(q(3)*q(4)+q(1)*q(2));
T(3,3)=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
%==========================================================================
end