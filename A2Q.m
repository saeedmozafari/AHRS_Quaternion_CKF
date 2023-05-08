function     q=A2Q(Euler)
% A2Q        := convert euler Angles to Quaternions vector
%==========================================================================
% q     (4*1):= quaternion vector
% psi   (1,1):= psi Angle
% theta (1,1):= theta Angle
% phi   (1,1):= phi Angle
% Euler   (1,3):= Euler Angles [psi, theta, phi]
%==========================================================================
psi = Euler(1);
theta = Euler(2);
phi = Euler(3);
q = zeros(4,1);
q(1) = cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
q(2) = cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);
q(3) = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);
q(4) = sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);
%==========================================================================
end