function [xhatkp,Pkp]=KalFil(wBLBkm1,ST,xhatkm1p,Pkm1p,yk_q,Qc,Rk)
%=================================================================================================
% Linear Continues Model x' = Fc*x + Lc*w
%==========================================================================
L=[-xhatkm1p(2) -xhatkm1p(3) -xhatkm1p(4);
    xhatkm1p(1) -xhatkm1p(4) xhatkm1p(3);
    xhatkm1p(4) xhatkm1p(1) -xhatkm1p(2);
    -xhatkm1p(3) -xhatkm1p(2) xhatkm1p(1)];
Lc=0.5*L;
%=================================================================================================
% Time Update
% attention: if Qk=Q, i.e. Q=constant, then calculation of Lkm1*Qkm1*Lkm1.'
%            should be performed only one time in the initialization. 
[xhatkn,Fkm1]=Quat_Propag(wBLBkm1,xhatkm1p,ST);
Qkm1_tilda=Lc*Qc*Lc'*ST;
Pkn=Fkm1*Pkm1p*Fkm1.'+Qkm1_tilda; % Qkm1_tilda ~= Lkm1*Qkm1*Lkm1.'
%==========================================================================
% y(k)=H(k)*x(k)+v(k)
Hk=eye(4);
%==========================================================================
% Measurement Update
Kk=Pkn*Hk.'/(Hk*Pkn*Hk.'+Rk);
xhatkp=xhatkn+Kk*(yk_q-Hk*xhatkn);
Pkp=(eye(4)-Kk*Hk)*Pkn;
%==========================================================================
end