%% System Parameters


 global r_PC_p; % distance from P to C in p frame
 r_PC_p=[1;0;0];

%r_PC_p= sym('r_PC_p',[3 1],'real')

global r_CB_c; % distance from C to B in c frame
r_CB_c=[0;0;0];

global etap_c; % eular angles of frame p w.r.t frame c
etap_c=[0;0;0];%[0*pi/3;0*pi/4;pi/4];

global etadp_c; % rate of eular angles of frame p w.r.t frame c
etadp_c=[0;0;0];

global etaddp_c; % rate of the derivative of the eular angles of frame p w.r.t frame c
etaddp_c=[0;0;0];

% global density_w; % density of water
% density_w=1.02; %kd/dm^3
syms density_w real

% global folaga_length;
% folaga_length=2.000; %% m
syms folaga_length real

% global folaga_diameter;
% folaga_diameter=0.155; %% m
syms folaga_diameter real
%folaga_diameter

global vol_auv; % volume of the AUV

vol_auv=pi*(folaga_diameter/2)^2*folaga_length; %%m^3

global buoyancy_max_auv;
buoyancy_max_auv=vol_auv*density_w*1000;

%global m_auv;
%m_auv=buoyancy_max_auv+.10;

global m_auv
syms m_auv real


global I_auv; % Inertia of the AUV w.r.t centroid and expressed in the c frame
I_auv=diag([1/2*m_auv*(folaga_diameter/2)^2 m_auv/12*(3*(folaga_diameter/2)^2+folaga_length^2) m_auv/12*(3*(folaga_diameter/2)^2+folaga_length^2)]); %x axis of the cylinder

global no_of_links; % number of links in the streamer
no_of_links=1;

%global link_length;
%link_length=1.6;
syms link_length real


% global link_diameter;
% link_diameter=.035;
syms link_diameter real

global rpipi_1_pi; % vector from pi to pi_1 [read as p_(i-1) frame] from frame pi
rpipi_1_pi=kron([link_length; 0; 0],ones(1,no_of_links)); 

global rpici_pi; % vector from pi to ci (centroid) from frame pi of the i-th link
rpici_pi=kron([link_length/2; 0; 0],ones(1,no_of_links)); 

global rpibi_pi; % vector from pi to bi (point of bouyancy) from frame pi of the i-th link
rpibi_pi=kron([link_length/2; 0; 0],ones(1,no_of_links)); 

global vol_link_ind; % volume of the AUV
vol_link_ind=pi*(link_diameter/2)^2*link_length; %%m^3

global buoyancy_max_link;
buoyancy_max_link=vol_link_ind*density_w*1000;

%buoyancy_max_link=buoyancy_max_auv

%global m_link_ind; % mass of the links
%m_link_ind=buoyancy_max_link+.1;
global m_link_ind
syms m_link_ind real

%m_link_ind=m_auv

global m_links
m_links=kron(m_link_ind,ones(no_of_links,1));

global I_links; % Inertia of the links w.r.t centroid of the i-th link and expressed in the frame parallel to {p_i} and with origin in C_i
I_link_ind=diag([1/2*m_link_ind*(link_diameter/2)^2 m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2)]); %x axis of the cylinder
%I_link_ind=diag([m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2) m_link_ind/12*(3*(link_diameter/2)^2+link_length^2)]); %x axis of the cylinder

I_links=repmat(I_link_ind,[1 1 no_of_links]);

global vol_links; % volume of the links
vol_links=kron(vol_link_ind,ones(no_of_links,1));

%global g; % gravitational acceleration
%g=[0;0;9.81];
global g0
syms g0 real
global g; % gravitational acceleration
g=[0;0;g0];





