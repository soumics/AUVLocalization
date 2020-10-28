clc;
close all;
clear all;
% hold on;

Param

uc_c=1; vyc_c=0; wc_c=0; pic_c=0; qic_c=0; ric_c=0; % linear and angular speed

omega_rel_single=[0;0;0];
omega_rel=kron(ones(no_of_links,1),omega_rel_single);

omega_rel(3)=pi/10;

u=[uc_c; vyc_c; wc_c; pic_c; qic_c; ric_c;omega_rel];

rc_i0=[1;1;1]; % initial position of point C

etac_i0=[0;0;0]; % Eular angles of C w.r.t I

eta_rel=kron(ones(no_of_links,1),[0;0;0]); 


q0=[rc_i0;etac_i0;eta_rel];

q=q0;

T=.1;

for i=1:10000
    t=i*T;
    if t<=100
        u(5)=pi/10;
    end
    if t>100
        u(5)=0;
        u(6)=pi/10;
    end
    qdot=kin_trans(q(:,i),u);    
    q(:,i+1)=qdot*T+q(:,i);
    
    [r,R]=pos_rot_pnts(q(:,i));
    cla
    hold on
    DrawPlot(r,R)
    hold off
    
    
    
    axis([-10 10 -10 10 -10 10])
    axis square
    grid on
    % view([0,90])
    %hold on;
    pause(.01)
    
end



