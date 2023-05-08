function     Euler=Q2A(q)
% Q2A        := convert Quaternions vector to euler Angles 
%==========================================================================
% q      (4*1):= quaternion vector
% psi    (1,1):= psi Angle
% theta  (1,1):= theta Angle
% phi    (1,1):= phi Angle
% Euler  (3,1):= Euler Angles [psi, theta, phi]
%==========================================================================
q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
%
Cbn_1_1 = q0^2+q1^2-q2^2-q3^2;
Cbn_1_2 = 2*(q1*q2+q0*q3);
Cbn_1_3 = 2*(q1*q3-q0*q2);
Cbn_2_3 = 2*(q2*q3+q0*q1);
Cbn_3_3 = q0^2-q1^2-q2^2+q3^2;
%
psi   = atan2(Cbn_1_2,Cbn_1_1);
theta = -atan2(Cbn_1_3,sqrt(1-Cbn_1_3^2));
phi   = atan2(Cbn_2_3,Cbn_3_3);
Euler = [psi; theta; phi];
%==========================================================================
end