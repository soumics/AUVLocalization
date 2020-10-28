%% Validation Test File

clc;
clear all;
close all;

% %% Validation of function 'pos_of_points'
% 
% rI_i=[2;0;0];
% etai_i=[0;0;0];
% R(:,:,1)=Rot_tot(etai_i);
% 
% rCI_c=[1;0;0];
% etac_i=0*[pi/6;0*pi/6;0*pi/6];
% R(:,:,2)=R(:,:,1)*Rot_tot(etac_i);
% rIC_i=-R(:,:,2)*rCI_c;
% rC_i=pos_of_points(rI_i,rIC_i,etai_i);
% 
% r=[rI_i rC_i];
% 
% figure
% hold on
% Drawplot(r,R)
% axis equal
% 
% %% Validation of function 'pos_rot'
% 
% [rC_i,R(:,:,2)]=pos_rot(rI_i,rCI_c,etai_i,etac_i);
% r=[rI_i rC_i];
% 
% figure
% hold on
% Drawplot(r,R)
% axis equal
% 
% %% Validation of function 'pos_rot_pnts'
% 
% Param
% etapi_1_pi_single=[0*pi/3;0*pi/3;0*pi/3];
% etapi_1_pi=kron(ones(no_of_links,1),etapi_1_pi_single);
% % etapi_1_pi(5)=pi/4;
% % etapi_1_pi(5)=pi/10;
% 
% q=[rC_i;etac_i;etapi_1_pi]; 
% [r,R]=pos_rot_pnts(q);
% 
% 
% figure
% hold on
% Drawplot(r,R)
% axis equal
% 
% 
% %% Validation of function 'vel_angvel'
% 
% vc_c=0*[0*1;0*1;1];
% etad_c=[0;0;0];
% 
% etadpi_1_pi_single=[0;0;0];
% etadpi_1_pi=kron(ones(no_of_links,1),etadpi_1_pi_single); 
% % etadpi_1_pi(1)=pi/6;
% 
% qdot=[vc_c;etad_c;etadpi_1_pi];
% [V,Omega]=vel_angvel(q,qdot)
% 
% %% Validation of function 'acc_angacc'
% 
% vdc_c=0*[0*1;0*1;1];
% etadd_c=[0;0;0];
% 
% etaddpi_1_pi_single=[0;0;0];
% etaddpi_1_pi=kron(ones(no_of_links,1),etaddpi_1_pi_single); 
% etaddpi_1_pi(1)=pi/6;
% 
% qddot=[vdc_c;etadd_c;etaddpi_1_pi];
% [Vd,Omegad]=acc_angacc(q,qdot,qddot)

%% Validation of function 'force_torque'

Param;
% udc_c=1.0; vydc_c=0; wdc_c=0; pdic_c=0; qdic_c=0; rdic_c=0; % linear and angular speed
% 
% omegad_rel_single=[0;0;0];
% omegad_rel=kron(ones(no_of_links,1),omegad_rel_single);

% omegad_rel(3)=pi/50;
% omegad_rel(6)=pi/50;

% ud=[udc_c; vydc_c; wdc_c; pdic_c; qdic_c; rdic_c;omegad_rel];


rc_i0=[1;1;1]; % initial position of point C

etac_i0=[pi/4;0;0]; % Eular angles of C w.r.t I

eta_rel=kron(ones(no_of_links,1),[0;0;0]);
%eta_rel(5)=pi/3;

vc_i0=[0;0;0]; % initial velocity of point C

etadc_i0=[0;0;0]; % Eular rate of C w.r.t I

etad_rel=kron(ones(no_of_links,1),[0;0;0]); % derivative of relative Eular rate

vdc_i0=[0;0;0]; % initial acceleration of point C

%etaddc_i0=[0;pi/3;0]; % initial derivative of Eular rate of C w.r.t I
etaddc_i0=[0;0;0]
etadd_rel=kron(ones(no_of_links,1),[0;0;0]); % second derivative of relative Eular rate
%etadd_rel(5)=1;

q0=[rc_i0;etac_i0;eta_rel];
qd0=[vc_i0;etadc_i0;etad_rel];
qdd0=[vdc_i0;etaddc_i0;etadd_rel];
q=q0; qdot=qd0; qddot=qdd0;

[r,R]=pos_rot_pnts(q)

q=[0 0 1 .7854 0 .7854]'
qdot=[0 0 0 0 0 0]'
qddot=[0 0 0 0 0 0]'
%qdot=[-0.1313   -0.0184   -2.6697   -0.2978    0.0677   -0.0043    0.3644    0.2038   -0.1306]'

%qddot=[-17.8144   -3.9374   -2.9539  131.0306   40.3550  115.5567   -7.4249  -42.7765   52.1476]'

% figure
% hold on
% grid on
% DrawPlot(r,R)
% axis([-10 10 -10 10 -10 10])
% set(gca, 'XDir', 'reverse')
% set(gca, 'ZDir', 'reverse')

[T,Tau]=force_torque(q,qdot,qddot)




