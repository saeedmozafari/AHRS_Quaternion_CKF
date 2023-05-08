close all
clear
clc
format compact
format short g
%==========================================================================
%Choose scenario
scenario=3;
% scenario = 1      Stationary        (60 seconds)
% scenario = 2      Ground path       (175 seconds)
% scenario = 3      Arial path        (220 seconds)
if scenario==1
    load('path1.mat')
end
if scenario==2
    load('path2.mat')
end
if scenario==3
    load('path3.mat')
end
%==========================================================================
% Constants
D2R=pi/180;
R2D=180/pi;
g=9.79;
%==========================================================================
ST=.01;
TF=(size(True.SBLL',1)*ST)-0.01;
%==========================================================================
%pick up the IMU.
IMU_Type = 3;
%                   Type      Grade
% IMU_Type = 1        MEMS      consumer
% IMU_Type = 2        MEMS      low-end tactical
% IMU_Type = 3        FOG       navigation
% IMU_Type = 4        FOG/RLG   strategic
[IMU] = Select_IMU(IMU_Type);
%==========================================================================
GPSF=10;    % GPS Frequency (Hz)
GPSU=(1/ST)/GPSF;   % GPS Update
%GPS velocity (@NED)
GPS_vBELkm1=zeros(3,1);
GPS_vBELk=zeros(3,1);
%==========================================================================
Error0=[3;1;1]*D2R;
Est.Euler=True.EULER(:,1)+Error0;
Est.xhat0p=A2Q(Est.Euler);
% x = [q0,q1,q2,q3]
Est.P0p=diag(A2Q(Error0.^2));
Est.Qc=eye(3)*(IMU.gyro_ARW/sqrt(ST))^2;
Est.Rk=diag ([1e-3   3e-4   3e-4   3e-4]);     %(1*D2R)^2~=3e-4 & (3*D2R)^2~=3e-3
%==================================
Est.xhatkm1p=Est.xhat0p;	% x_hat_k_minus_1_positive
Est.Pkm1p=Est.P0p;	        % P_k_minus_1_positive
%==================================
Est.Xhatp=Est.xhat0p;
Est.Pp=diag(Est.P0p);
Y_e=zeros(3,1);
Y_q=zeros(4,1);

%quantization error
qra = zeros(3,1);
qrg = zeros(3,1);
qrm = zeros(3,1);

% buffer the last two records of heading
psi_buffer=[True.EULER(1,1)+Error0(1); True.EULER(1,1)+Error0(1)];
%==========================================================================
for k=1:1:TF/ST      % k=0 means t=0 & k=1 means t=0.01 & ...
    %======================================================================
    %################################AHRS#################################%
    %======================================================================
    % True first elements refer to initial conditions!
    %   axkm1=True.ABLB_ng(1,k);    % True.ABLB_ng(1,1) means ax0
    %   aykm1=True.ABLB_ng(2,k);    % True.ABLB_ng(2,1) means ay0
    %   azkm1=True.ABLB_ng(3,k);    % True.ABLB_ng(3,1) means az0
    
    [meas_fBIB,qra] = acc_model(ST,True.ABLB_ng(:,k+1),IMU,qra);
    axk=meas_fBIB(1);    % True.ABLB_ng(1,2) means ax1
    ayk=meas_fBIB(2);    % True.ABLB_ng(2,2) means ay1
    azk=meas_fBIB(3);    % True.ABLB_ng(3,2) means az1
    
    [meas_wBIB,qrg] = Gyro_model(ST,True.ABLB_ng(:,k),True.WBLB(:,k),IMU,qrg);
    pkm1=meas_wBIB(1);        % True.WBLB(1,1) means p0
    qkm1=meas_wBIB(2);        % True.WBLB(2,1) means q0
    rkm1=meas_wBIB(3);        % True.WBLB(3,1) means r0
    
    wBLBkm1=[pkm1;qkm1;rkm1];
    
    [meas_wBIB,qrg] = Gyro_model(ST,True.ABLB_ng(:,k+1),True.WBLB(:,k+1),IMU,qrg);
    pk=meas_wBIB(1);        % True.WBLB(1,2) means p1
    qk=meas_wBIB(2);        % True.WBLB(2,2) means q1
    rk=meas_wBIB(3);        % True.WBLB(3,2) means r1
    
    wBLBk=[pk;qk;rk];
    %======================================================================
    if mod(k,GPSU)==0
        if k/GPSU==1
            GPS_vBELk=GPS_model(True.VBLL(:,k));
            GPS_vBELkm1=GPS_vBELk;
        else
            GPS_vBELk = GPS_model(True.VBLL(:,k));
        end
    end
    
    TBLkm1p=Q2TM(Est.xhatkm1p);
    VBLBkm1p=TBLkm1p * GPS_vBELkm1;
    ukm1=VBLBkm1p(1);        % means u0
    vkm1=VBLBkm1p(2);        % means v0
    wkm1=VBLBkm1p(3);        % means w0

    [xhatkn,~]=Quat_Propag(wBLBkm1,Est.xhatkm1p,ST);% Propagate one step
    TBLkn=Q2TM(xhatkn);    
    VBLBkn=TBLkn * GPS_vBELk;
    uk=VBLBkn(1);        %  means u1
    vk=VBLBkn(2);        %  means v1
    wk=VBLBkn(3);        %  means w1
    
    GPS_vBELkm1=GPS_vBELk;
    %======================================================================
%         udk=(uk-ukm1)/ST;
%         vdk=(vk-vkm1)/ST;
%         wdk=(wk-wkm1)/ST;
    
    udk=0;
    vdk=0;
    wdk=0;
    %======================================================================
    % theta_m=asin(ax/GEMT.g);
    % phi_m=asin(ay/(-GEMT.g*cos(theta_m)));
    % phi_m=atan(ay/az/GEMT.g);
    % Measurments=[Measurments [0;theta_m;phi_m]];
    %======================================================================
    theta_m=asin((udk-vk*rk+wk*qk-axk)/-g);
    %     phi_m=atan((vdk+uk*rk-wk*pk-ayk)/(wdk-uk*qk+vk*pk-azk));
    phi_m=asin((vdk+uk*rk-wk*pk-ayk)/(g*cos(theta_m)));
    %======================================================================
    [M_L,~,d,~,~]=igrfmagm(True.SBLL(3,k+1),True.SBLL(1,k+1)* R2D,True.SBLL(2,k+1)* R2D,2011);
    D=d*D2R;    % D will be read from lookup-table
    [meas_mag,qrm] = Mag_model(ST,M_L',IMU,qrm);
    
    TBLt=Q2TM(True.QUAT(:,k+1));
    M_B=TBLt*meas_mag;
    mx=M_B(1);
    my=M_B(2);
    mz=M_B(3);
   
    dpsi_m=atan2((-cos(phi_m)*my+sin(phi_m)*mz),(cos(theta_m)*mx+sin(theta_m)*sin(phi_m)*my+sin(theta_m)*cos(phi_m)*mz));
    %heading signal phase unwraping
    psi_buffer=[psi_buffer(2);dpsi_m+D];
    psi_buffer = Phase_Unwrap(psi_buffer,pi);
    psi_m=psi_buffer(2);
    %======================================================================
    yk_e=[psi_m;theta_m;phi_m];
    yk_q=A2Q(yk_e);
    Y_e=[Y_e yk_e];
    Y_q=[Y_q yk_q];
    %======================================================================
    [Est.xhatkp,Est.Pkp]=KalFil(wBLBkm1,ST,Est.xhatkm1p,Est.Pkm1p,yk_q,Est.Qc,Est.Rk);
    %======================================================================
    % Normalization
    Est.xhatkp=quatnormz(Est.xhatkp);
    %==============================================
    Est.xhatkm1p = Est.xhatkp;  %for next iteration
    Est.Pkm1p=Est.Pkp;          %for next iteration
    %==============================================
    Est.Xhatp=[Est.Xhatp Est.xhatkp];
    Est.Pp=[Est.Pp diag(Est.Pkp)];
    Est.Euler=[Est.Euler, Q2A(Est.xhatkp)];
    %==========================================================================
end
%================================================================================================================================================================
% t=ST:ST:TF;
% figure;plot(t,True.VBLL(1,:));xlabel('time (s)');ylabel('v_N (m/s)');
% figure;plot(t,True.VBLL(2,:));xlabel('time (s)');ylabel('v_E (m/s)');
% figure;plot(t,True.VBLL(3,:));xlabel('time (s)');ylabel('v_D (m/s)');
% figure;plot(t,True.SBLL(1,:));xlabel('time (s)');ylabel('s_N (m)');
% figure;plot(t,True.SBLL(2,:));xlabel('time (s)');ylabel('s_E (m)');
% figure;plot(t,-True.SBLL(3,:));xlabel('time (s)');ylabel('h (m)');
% figure;plot(t,True.EULER(1,:)*R2D);xlabel('time (s)');ylabel('\psi (deg)');
% figure;plot(t,True.EULER(2,:)*R2D);xlabel('time (s)');ylabel('\theta (deg)');
% figure;plot(t,True.EULER(3,:)*R2D);xlabel('time (s)');ylabel('\phi (deg)');
%==========================================================================
t=0:ST:TF;
figure;plot(t,True.SBLL(3,:));xlabel('time (s)');ylabel('h (m)')
figure;plot(t,True.QUAT(1,:),'r',t,Y_q(1,:),'b-.',t,Est.Xhatp(1,:),'g--');
xlabel('time (s)');ylabel('q0');legend('True','Measured','Estimated')
figure;plot(t,True.QUAT(2,:),'r',t,Y_q(2,:),'b-.',t,Est.Xhatp(2,:),'g--');
xlabel('time (s)');ylabel('q1');legend('True','Measured','Estimated')
figure;plot(t,True.QUAT(3,:),'r',t,Y_q(3,:),'b-.',t,Est.Xhatp(3,:),'g--');
xlabel('time (s)');ylabel('q2');legend('True','Measured','Estimated')
figure;plot(t,True.QUAT(4,:),'r',t,Y_q(4,:),'b-.',t,Est.Xhatp(4,:),'g--');
xlabel('time (s)');ylabel('q3');legend('True','Measured','Estimated')
%==========================================================================
figure;plot(t,True.EULER(1,:)*R2D,'r',t,Y_e(1,:)*R2D,'b-.',t,Est.Euler(1,:)*R2D,'g--');
xlabel('time (s)');ylabel('\psi (deg)');legend('True','Measured','Estimated')
figure;plot(t,True.EULER(2,:)*R2D,'r',t,Y_e(2,:)*R2D,'b-.',t,Est.Euler(2,:)*R2D,'g--');
xlabel('time (s)');ylabel('\theta (deg)');legend('True','Measured','Estimated')
figure;plot(t,True.EULER(3,:)*R2D,'r',t,Y_e(3,:)*R2D,'b-.',t,Est.Euler(3,:)*R2D,'g--');
xlabel('time (s)');ylabel('\phi (deg)');legend('True','Measured','Estimated')
%==========================================================================