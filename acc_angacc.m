%% This function calculates the linear and angular acceleration of the origin of
%% all the frames given the states (auv position, orientation, eular angles
%% and all of their [aforementioned states] single and double derivatives respectively)
%% Function Arguments:
%% q=states (auv position, orientation,eular angles
%% qdot=derivative of q
%% qddot=derivative of qdot
%% and all of their [aforementioned states] single and double derivatives respectively)
%% Function Returns:
%% Vd=linear acceleration of all the origins of the frames
%% Wd=angular acceleration of all the origins of the frames
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P,R,V,W,Vd,Wd]=acc_angacc(q,qdot,qddot)

global r_PC_p;
global etap_c;
global etadp_c;
global etaddp_c;
global rpipi_1_pi;
global no_of_links;

%cen_vec_len=6;
%init_loc_q=cen_vec_len+1;
%init_loc_qd=cen_vec_len+1;
%init_loc_qdd=cen_vec_len+1;

%vc_c=qdot(1:3); % velocity of C in frame C
rC_i=q(1:3);
vc_i=qdot(1:3); % velocity of C in frame C
vdc_i=qddot(1:3);

eta_c=q(4:6); % Eular angles of C w.r.t I
etad_c=qdot(4:6); % angular velocity of C in frame C
etadd_c=qddot(4:6);

%%%%%%%%%%%%%%%%%% Kinematics of Point C %%%%%%%%%%%%%%%%%%%%%%%%%

Rc_i=Rot_tot(eta_c); % Rotation of C w.r.t I
T_etac_i=Ang_trans(eta_c); % angular transformation of C w.r.t I
inv_T_etac_i=Ang_trans_inv(eta_c); % angular transformation of C w.r.t I
%rC_i,r_PC_p,eta_c,etap_c
[rP_i,Rp_i]=pos_rot(rC_i,r_PC_p,eta_c,etap_c);
P=[rC_i rP_i];



%vc_c=Rc_i'*vc_i; % velocity of C in I

omega_ic_c=inv_T_etac_i*etad_c; % Eular rate of C in I
omega_ic_i=Rc_i*omega_ic_c; % angular velocity of C in I

Somega_ic_i=Smtrx(omega_ic_i);
%vdc_i=Rc_i*vdc_c+Rc_i*Somega_ic_i*vc_c;


%Tdc_i=Ang_transd([eta_c,etad_c]);
%%omegad_ic_c=inv(Tc_i)*(etadd_c-Tdc_i*inv(Tc_i)*etad_c);
%omegad_ic_i=Rc_i*omegad_ic_c+Rc_i*Somega_ic_i*omega_ic_c
%Ang_transd_inv([eta_c,etad_c])
omegad_ic_i=Somega_ic_i*Rc_i*inv_T_etac_i*etad_c+Rc_i*Ang_transd_inv([eta_c,etad_c])*etad_c+Rc_i*inv_T_etac_i*etadd_c;

%%%%%%%%%%%%%%%%%% Kinematics of Point P %%%%%%%%%%%%%%%%%%%%%%%%%
Rp_c=Rot_tot(etap_c);
Tp_c=Ang_trans(etap_c);
inv_Tp_c=Ang_trans_inv(etap_c);
Tdp_c=Ang_transd([etap_c;etadp_c]);
Rp_i=Rc_i*Rp_c;
omega_cp_p=inv_Tp_c*etadp_c;
omega_cp_c=Rp_c*omega_cp_p;
omega_ip_i=omega_ic_i+Rp_i*omega_cp_p;

vp_i=vc_i+cross(omega_ip_i,-Rp_i*r_PC_p);

%omegad_cp_c=inv(Tp_c)*(etaddp_c'-Tdp_c*inv(Tp_c)*etadp_c'):
omegad_cp_c=inv_Tp_c*etaddp_c+Ang_transd_inv([etap_c,etadp_c])*etadp_c;
%-inv(Tp_c)*Tdp_c*inv(Tp_c)*etadp_c'):
omegad_ip_i=omegad_ic_i+Rc_i*omegad_cp_c+Somega_ic_i*Rc_i*omega_cp_c;
Somega_ip_i=Smtrx(omega_ip_i);
Somegad_ip_i=Smtrx(omegad_ip_i);
vdp_i=vdc_i-Somegad_ip_i*Rp_i*r_PC_p-Somega_ip_i^2*Rp_i*r_PC_p;

R(:,:,1)=Rc_i;
R(:,:,2)=Rp_i;
eta_p=Eular_ang(Rp_i);



V=[vc_i vp_i];
W=[omega_ic_i omega_ip_i];
Vd=[vdc_i vdp_i];
Wd=[omegad_ic_i omegad_ip_i];


%%%%%%%%%%%%%%%%%% Kinematics of other points %%%%%%%%%%%%%%%%%%%%%%%%%

R_pi_1_i=Rp_i;
omega_pi_1_i=omega_ip_i;
v_pi_1_i=vp_i;
%Somega_pi_1=Smtrx(omega_pi_1_i);
omegad_pi_1_i=omegad_ip_i;
vd_pi_1_i=vdp_i;
rpi_1_i=rP_i;
eta_pi_1=eta_p;

for i=1:no_of_links
    eta_pi_pi_1=q(7+3*(i-1):6+3*i);
    %  rpi_1_i,rpipi_1_pi(i,:)',eta_pi_1,eta_pi_1_pi
    % rpi_1_i,rpipi_1_pi(i,:)',eta_pi_1,eta_pi_1_pi
    [rpi_i,R_pi_i]=pos_rot(rpi_1_i,rpipi_1_pi(:,i),eta_pi_1,eta_pi_pi_1);
    R_pi_pi_1=Rot_tot(eta_pi_pi_1);
    %R_pi_i=R_pi_1_i*R_pi_pi_1;
    
    P=[P rpi_i];
    
    
    eta_pi=Eular_ang(R_pi_i);
    T_eta_pi_pi_1=Ang_trans(eta_pi_pi_1);
    inv_T_eta_pi_pi_1=Ang_trans_inv(eta_pi_pi_1);
    
    etad_pi_pi_1=qdot(7+3*(i-1):6+3*i);
    omega_pi_1_pi_pi=inv_T_eta_pi_pi_1*etad_pi_pi_1;
    omega_pi_i=omega_pi_1_i+R_pi_i*omega_pi_1_pi_pi;
    Somega_pi=Smtrx(omega_pi_i);
    
    etadd_pi_pi_1=qddot(7+3*(i-1):6+3*i);
    
    
    %      omegad_pi_1_i
    %      Smtrx(omega_pi_i)*R_pi_i*inv_T_eta_pi_1_pi*etad_pi_1_pi
    %      R_pi_i*Ang_transd_inv([eta_pi_1_pi,etad_pi_1_pi])*etad_pi_1_pi
    %      R_pi_i*inv_T_eta_pi_1_pi*etadd_pi_1_pi%)+R_curr*Somega_curr*omega_rel;
    
    
    
    R(:,:,2+i)=R_pi_i;
    W=[W omega_pi_i];
    
    
    v_pi_i=v_pi_1_i+cross(omega_pi_i,-R_pi_i*rpipi_1_pi(:,i));
    
    omegad_pi_i=omegad_pi_1_i+Smtrx(omega_pi_i)*R_pi_i*inv_T_eta_pi_pi_1*etad_pi_pi_1+R_pi_i*Ang_transd_inv([eta_pi_pi_1,etad_pi_pi_1])*etad_pi_pi_1+R_pi_i*inv_T_eta_pi_pi_1*etadd_pi_pi_1;%)+R_curr*Somega_curr*omega_rel;
    Somegad_pi=Smtrx(omegad_pi_i);
    vd_pi_i=vd_pi_1_i+Somegad_pi*R_pi_i*(-rpipi_1_pi(:,i))+Somega_pi^2*R_pi_i*(-rpipi_1_pi(:,i));
    V=[V v_pi_i];
    Wd=[Wd omegad_pi_i];
    
    Vd=[Vd vd_pi_i];
    
    eta_pi_1=eta_pi;
    rpi_1_i=rpi_i;
    R_pi_1_i=R_pi_i;
    v_pi_1_i=v_pi_i;
    vd_pi_1_i=vd_pi_i;
    omega_pi_1_i=omega_pi_i;
    %  Somega_pi_1=Somega_pi;
    omegad_pi_1_i=omegad_pi_i;
end















