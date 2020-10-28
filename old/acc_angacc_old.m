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

function [Vd,Wd]=acc_angacc(q,qdot,qddot)

global r_PC_p;
global etap_c;
global etadp_c;
global etaddp_c;
global rpipi_1_pi;
global no_of_links;

cen_vec_len=6;
init_loc_q=cen_vec_len+1;
init_loc_qd=cen_vec_len+1;
init_loc_qdd=cen_vec_len+1;

%vc_c=qdot(1:3); % velocity of C in frame C

etad_c=qdot(4:6); % angular velocity of C in frame C

%vdc_c=qddot(1:3);
etadd_c=qddot(4:6);

%%%%%%%%%%%%%%%%%% Kinematics of Point C %%%%%%%%%%%%%%%%%%%%%%%%%

eta_c=q(4:6); % Eular angles of C w.r.t I
Rc_i=Rot_tot(eta_c); % Rotation of C w.r.t I
Tc_i=Ang_trans(eta_c); % angular transformation of C w.r.t I

vc_i=qdot(1:3); % velocity of C in frame C
%vc_c=Rc_i'*vc_i; % velocity of C in I

omega_ic_c=inv(Tc_i)*etad_c; % Eular rate of C in I
omega_ic_i=Rc_i*omega_ic_c; % angular velocity of C in I

Somega_ic_i=Smtrx(omega_ic_i);
%vdc_i=Rc_i*vdc_c+Rc_i*Somega_ic_i*vc_c;
vdc_i=qddot(1:3);


Tdc_i=Ang_transd([eta_c,etad_c]);
%%omegad_ic_c=inv(Tc_i)*(etadd_c-Tdc_i*inv(Tc_i)*etad_c);
%omegad_ic_i=Rc_i*omegad_ic_c+Rc_i*Somega_ic_i*omega_ic_c
%Ang_transd_inv([eta_c,etad_c])
omegad_ic_i=Somega_ic_i*Rc_i*inv(Tc_i)*etad_c+Rc_i*Ang_transd_inv([eta_c,etad_c])*etad_c+Rc_i*inv(Tc_i)*etadd_c;

%%%%%%%%%%%%%%%%%% Kinematics of Point P %%%%%%%%%%%%%%%%%%%%%%%%%
Rp_c=Rot_tot(etap_c);
Tp_c=Ang_trans(etap_c);
omega_cp_c=inv(Tp_c)*etadp_c';
omega_ip_i=omega_ic_i+Rc_i*omega_cp_c;
Rp_i=Rc_i*Rp_c;
vp_i=vc_i-cross(omega_ip_i,Rp_i*r_PC_p);

Tdp_c=Ang_transd([etap_c;etadp_c]);
omegad_cp_c=inv(Tp_c)*(etaddp_c'-Tdp_c*inv(Tp_c)*etadp_c');
omegad_ip_i=omegad_ic_i+Rc_i*omegad_cp_c+Rc_i*Somega_ic_i*omega_cp_c;
Somegad_ip_i=Smtrx(omegad_ip_i);
Somega_ip_i=Smtrx(omega_ip_i);
vdp_i=vdc_i-Somegad_ip_i*Rp_i*r_PC_p-Somega_ip_i^2*Rp_i*r_PC_p;


%%%%%%%%%%%%%%%%%% Kinematics of other points %%%%%%%%%%%%%%%%%%%%%%%%%

R_curr=Rp_i;
omega_curr=omega_ip_i;
v_curr=vp_i;
Somega_curr=Smtrx(omega_curr);
omegad_curr=omegad_ip_i;
vd_curr=vdp_i;

Vd=[vdc_i vd_curr];
Wd=[omegad_ic_i omegad_curr];

for i=1:no_of_links
    eta_rel=q(init_loc_q+3*(i-1):init_loc_q+3*i-1)';
    R_rel=Rot_tot(eta_rel);
    T_rel=Ang_trans(eta_rel);
    etad_rel=qdot(init_loc_qd+3*(i-1):init_loc_qd+3*i-1)';
    omega_rel=inv(T_rel)*etad_rel';
    omega_next=omega_curr+R_curr*omega_rel;
    etadd_rel=qddot(init_loc_qdd+3*(i-1):init_loc_qdd+3*i-1)';
%     Td_rel=Ang_transd([eta_rel,etad_rel]);
%     omegad_rel=inv(T_rel)*(etadd_rel'-Td_rel*inv(T_rel)*etad_rel');
%     omegad_next=omegad_curr+R_curr*omegad_rel+R_curr*Somega_curr*omega_rel;
%     
 %   Td_rel=Ang_transd([eta_rel,etad_rel]);
   
 
 %omegad_rel=inv(T_rel)*(etadd_rel'+Ang_transd_inv([eta_rel,etad_rel])*etad_rel');
    
    
    %omegad_next=omegad_curr+R_curr*omegad_rel+R_curr*Somega_curr*omega_rel;
         
  %  omegad_curr
  %  R_curr
     R_next=R_curr*R_rel;
%    Smtrx(omega_rel)*R_rel*inv(T_rel)*etad_rel'
%    R_rel*Ang_transd_inv([eta_rel,etad_rel])*etad_rel'+R_rel*inv(T_rel)*etadd_rel'
%    R_curr*Somega_curr*omega_rel

%     omegad_next=omegad_curr+R_curr*(Smtrx(omega_rel)*R_rel*inv(T_rel)*etad_rel'+R_rel*Ang_transd_inv([eta_rel,etad_rel])*etad_rel'+R_rel*inv(T_rel)*etadd_rel')+R_curr*Somega_curr*omega_rel;

     omegad_next=omegad_curr+Smtrx(omega_next)*R_next*inv(T_rel)*etad_rel'+R_next*Ang_transd_inv([eta_rel,etad_rel])*etad_rel'+R_next*inv(T_rel)*etadd_rel';%)+R_curr*Somega_curr*omega_rel;

 %    omegad_ic_i=Somega_ic_i*Rc_i*inv(Tc_i)*etad_c+Rc_i*Ang_transd_inv([eta_c,etad_c])*etad_c+Rc_i*inv(Tc_i)*etadd_c;

     
 %   omegad_ic_i=Somega_ic_i*Rc_i*inv(Tc_i)*etad_c+Rc_i*Ang_transd_inv([eta_c,etad_c])*etad_c+Rc_i*inv(Tc_i)*etadd_c;
%    omegad_next=omegad_curr+R_curr*omegad_rel+R_curr*Somega_curr*omega_rel;

    Wd=[Wd omegad_next];
   
    
    v_next=v_curr+cross(omega_next,-R_next*rpipi_1_pi(i,:)');
    Somegad_next=Smtrx(omegad_next);
    Somega_next=Smtrx(omega_next);
    vd_next=vd_curr+Somegad_next*R_next*(-rpipi_1_pi(i,:)')+Somega_next^2*R_next*(-rpipi_1_pi(i,:)');
    Vd=[Vd vd_next];
    
    R_curr=R_next;
    omega_curr=omega_next;
    v_curr=v_next;
    Somega_curr=Somega_next;
    omegad_curr=omegad_next;
    vd_curr=vd_next;
end















