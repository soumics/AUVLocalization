clc;
clear all;
close all;
hold on;

Param

r_i=[0;0;0];
T00 = [eye(3) r_i; 0 0 0 1];
DrawFrame(T00,1);

etac_i=0*[0;0;pi/6];
Rc_i=Rot_tot(etac_i);
Pc_i=[1;1;0];

Tc = [Rc_i Pc_i; 0 0 0 1];
DrawFrame(Tc,1);

vc_c=[1;0;0];
etad_c=0*[1;0;0];

etapi_pi_1=kron(ones(no_of_links,1),0*[0;0;pi/3]);

etadpi_pi_1=kron(ones(no_of_links,1),0*[0;0;pi/6]); 

q=[Pc_i;etac_i;etapi_pi_1]; qdot=[vc_c;etad_c;etadpi_pi_1];
[r,R]=pos_rot_pnts(q);
[V,Omega]=vel_angvel([q;qdot]);

grid on;

r=[r_i r]
hold on
Drawplot(r,cat(3,Rc_i,R))

axis equal
% set(gca, 'XDir', 'reverse')
% set(gca, 'ZDir', 'reverse')
