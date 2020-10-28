%% This function calculates the linear and angular speed of the origin of
%% all the frames given the states (auv position, orientation,eular angles
%% and all of their [aforementioned states] derivatives)
%% Function Arguments:
%% q=states (auv position, orientation,eular angles
%% and all of their [aforementioned states] derivatives)
%% Function Returns:
%% V=linear velocity of all the origins of the frames
%% W=angular speed of all the origins of the frames
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [V,W]=vel_angvel(q,qdot)

global r_PC_p;
global etap_c;
global etadp_c;
global rpipi_1_pi;
global no_of_links;

cen_vec_len=6;
init_loc_q=cen_vec_len+1;
init_loc_qd=cen_vec_len+1;
%Rtot(q(1:3))
%vc_c=qdot(1:3); % velocity of C in frame C
etad_c=qdot(4:6); % Euler angles rate

%%%%%%%%%%%%%%%%%% Kinematics of Point C %%%%%%%%%%%%%%%%%%%%%%%%%

eta_c=q(4:6); % Eular angles of C w.r.t I
Rc_i=Rot_tot(eta_c); % Rotation of C w.r.t I
Tc_i=Ang_trans(eta_c); % angular transformation of C w.r.t I

%vc_i=Rc_i*vc_c; % velocity of C in I
vc_i=qdot(1:3);
omega_ic_c=inv(Tc_i)*etad_c; % Eular rate of C in I
omega_ic_i=Rc_i*omega_ic_c; % angular velocity of C in I

%%%%%%%%%%%%%%%%%% Kinematics of Point P %%%%%%%%%%%%%%%%%%%%%%%%%
Rp_c=Rot_tot(etap_c);
Tp_c=Ang_trans(etap_c);
omega_cp_c=inv(Tp_c)*etadp_c';
omega_ip_i=omega_ic_i+Rc_i*omega_cp_c;
Rp_i=Rc_i*Rp_c;
vp_i=vc_i+cross(omega_ip_i,-Rp_i*r_PC_p);


%%%%%%%%%%%%%%%%%% Kinematics of other points %%%%%%%%%%%%%%%%%%%%%%%%%
omega_curr=omega_ip_i;
R_curr=Rp_i;
v_curr=vp_i;
V=[vc_i v_curr];
W=[omega_ic_i omega_curr];

for i=1:no_of_links
    eta_r=q(6+3*(i-1)+1:6+3*i)';
    R_rel=Rot_tot(eta_r);
    T_rel=Ang_trans(eta_r);
    etad_rel=qdot(6+3*(i-1)+1:6+3*i);
    omega_rel=inv(T_rel)*etad_rel;
    omega_next=omega_curr+R_curr*omega_rel;
    W=[W omega_next];
    R_next=R_curr*R_rel;
    
    v_next=v_curr+cross(omega_next,-R_next*rpipi_1_pi(i,:)');
    V=[V v_next];
    
    R_curr=R_next;
    omega_curr=omega_next;
    v_curr=v_next;
end
















