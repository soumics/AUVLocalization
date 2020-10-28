%% System Parameters


global r_PC_p; % distance from P to C in p frame
r_PC_p=[1;0;0];

global r_CB_c; % distance from C to B in c frame
r_CB_c=[0;0;0];

global etap_c; % eular angles of frame p w.r.t frame c
etap_c=[0;0;0];%[0*pi/3;0*pi/4;pi/4];

global etadp_c; % rate of eular angles of frame p w.r.t frame c
etadp_c=[0;0;0];

global etaddp_c; % rate of the derivative of the eular angles of frame p w.r.t frame c
etaddp_c=[0;0;0];

global density_w; % density of water
density_w=1.02*1000; %kd/dm^3

global folaga_length;
folaga_length=2.000; %% m

global folaga_diameter;
folaga_diameter=0.155; %% m

% L/D_auv=12.9032 

global surf_area_auv_front;
surf_area_auv_front=pi*((folaga_diameter/2)^2);
global surf_area_auv_side;
surf_area_auv_side=folaga_diameter*folaga_length;

global vol_auv; % volume of the AUV
vol_auv=pi*(folaga_diameter/2)^2*folaga_length; %%m^3

global buoyancy_max_auv;
buoyancy_max_auv=vol_auv*density_w;

global m_auv;
m_auv=buoyancy_max_auv+0.01;
%+0.000000000001;

global I_auv; % Inertia of the AUV w.r.t centroid and expressed in the c frame
I_auv=diag([1/2*m_auv*(folaga_diameter/2)^2 m_auv/12*(3*(folaga_diameter/2)^2+folaga_length^2) m_auv/12*(3*(folaga_diameter/2)^2+folaga_length^2)]); %x axis of the cylinder

global no_of_links; % number of links in the streamer
no_of_links=8;

global link_length;
link_length=1.0;%6;

global link_diameter;
link_diameter=.035;

% L/D_link=45.7143 

global surf_area_link_front;
surf_area_link_front=pi*((link_diameter/2)^2);
global surf_area_link_side;
surf_area_link_side=link_diameter*link_length;

global rpipi_1_pi; % vector from pi to pi_1 [read as p_(i-1) frame] from frame pi
rpipi_1_pi=kron([link_length; 0; 0],ones(1,no_of_links));

global rpici_pi; % vector from pi to ci (centroid) from frame pi of the i-th link
rpici_pi=kron([link_length/2; 0; 0],ones(1,no_of_links));

global rpibi_pi; % vector from pi to bi (point of bouyancy) from frame pi of the i-th link
rpibi_pi=kron([link_length/2; 0; 0],ones(1,no_of_links));

global vol_link_ind; % volume of the AUV
vol_link_ind=pi*(link_diameter/2)^2*link_length; %%m^3

global buoyancy_max_link;
buoyancy_max_link=vol_link_ind*density_w;

%buoyancy_max_link=buoyancy_max_auv

global m_link_ind; % mass of the links
m_link_ind=buoyancy_max_link+0.01;
%+0.000000000001;
%m_link_ind=m_auv

global m_links
m_links=kron(m_link_ind,ones(no_of_links,1));
%m_links(1:no_of_links-1)=buoyancy_max_link+0.001;


global I_links; % Inertia of the links w.r.t centroid of the i-th link and expressed in the frame parallel to {p_i} and with origin in C_i
I_link_ind=diag([1/2*m_link_ind*(link_diameter/2)^2 m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2)]); %x axis of the cylinder
%I_link_ind=diag([m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2)]); %x axis of the cylinder

I_links=repmat(I_link_ind,[1 1 no_of_links]);

global vol_links; % volume of the links
vol_links=kron(vol_link_ind,ones(no_of_links,1));

global g; % gravitational acceleration
g=[0;0;9.81];

global fric_coeff;
fric_coeff=0.5*0;
global drag_coeff;
drag_coeff=0.8; %subcritical flow 0.82 %soumic 0.4;
global lift_coeff;
lift_coeff=0.7; %subcritical flow 0.7 %soumic 0.8

global fric_coeff_link;
fric_coeff_link=0.5*0; %subcritical flow
global drag_coeff_link;
drag_coeff_link=0.99; %subcritical flow 0.99 %soumic 0.4;
global lift_coeff_link;
lift_coeff_link=1.2; %subcritical flow 1.2 %soumic 0.8

global drag_mcoeff;
drag_mcoeff=0.1;

global water_speed;
water_speed=[-0*0.1;0;0*1.5];

global var_usbl;
var_usbl=1;
global var_dvl;
var_dvl=1;
global var_acc;
var_acc=1;
global var_gyro;
var_gyro=1;
global var_depth;
var_depth=0;

global var_releuang;
var_releuang=0;
global var_releurate;
var_releurate=0;

global XD;

%NO_EKF_corr=var_usbl|var_dvl|var_acc|var_gyro|var_depth
%NO_EKF_corr=0;

XD=[];


global sig_usbl;
sig_usbl=.2;
global sig_depth;
sig_depth=.1;
global sig_gyro;
sig_gyro=(2/180)*pi;

global sig_releuang;
sig_releuang=(2/180)*pi;
global sig_releurate;
sig_releurate=(2/180)*pi;


global sig_dvlx;
sig_dvlx=.1;
global sig_dvly;
sig_dvly=.1;
global sig_dvlz;
sig_dvlz=.1;
global sig_dvlphi;
sig_dvlphi=(2/180)*pi;
global sig_dvlth;
sig_dvlth=(2/180)*pi;
global sig_dvlpsi;
sig_dvlpsi=(2/180)*pi;

global sig_rng;
sig_rng=20;







