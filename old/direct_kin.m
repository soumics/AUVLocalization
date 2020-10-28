%% This function calculates the acceleration of the states given the states 'q',
%% the derivative of the states 'qdot' and the control input 'u', i.e., qddot=f(q,qdot,u)
function qddot=direct_dyn(q,qdot,u)

global no_of_links;
global m_auv;
global Ic_c;
global I_rel;

vc_i=qdot(1:3); % velocity of C in frame C
etadc_i=qdot(4:6); % angular velocity of C in frame C
etad_rel=q(7:6+3*no_of_links);

fc_c=u(1:3);
tauc_c=u(4:6);

tau_rel=u(7:6+3*no_of_links);  
%omegad_rel=   

%%%%%%%%%%%%%%%%%% Kinematics of Point C %%%%%%%%%%%%%%%%%%%%%%%%%

eta_c=q(4:6); % Eular angles of C w.r.t I
Rc_i=Rot_tot(eta_c); % Rotation of C w.r.t I
Tc_i=Ang_trans(eta_c); % angular transformation of C w.r.t I
vc_c=inv(Rc_i)*vc_i; % velocity of C in I
omega_ic_i=inv(Tc_i)*etadc_i; % Eular rate of C in I
omega_ic_c=inv(Rc_i)*omega_ic_i;

Somega_ic_c=Smtrx(omega_ic_c);
vdc_c=fc_c/m_auv-Somega_ic_c*vc_c;
omegad_ic_c=inv(Ic_c)*(tauc_c-Somega_ic_c*Ic_c*omega_ic_c);

Somega_ic_i=Smtrx(omega_ic_i);
vdc_i=Rc_i*vdc_c+Rc_i*Somega_ic_i*vc_c;
Tdc_i=Ang_transd([eta_c,etadc_i]);
etaddc_i=Tc_i*omegad_ic_c+Tdc_i*omega_ic_c;

%%%%%%%%%%%%%%%%%% Kinematics of other points %%%%%%%%%%%%%%%%%%%%%%%%%

%init=7;

etadd_rel=[];

for i=1:no_of_links
    eta_rel=q(6+3*(i-1)+1:6+3*i);
    T_rel=Ang_trans(eta_rel);
    etad_rel_var=etad_rel(3*(i-1)+1:3*i);
    omega_rel_var=inv(T_rel)*etad_rel_var;
    omegad_rel_var=inv(I_rel)*tau_rel(3*(i-1)+1:3*i);
    
    Td_rel=Ang_transd([eta_rel,etad_rel_var]);
    etadd_rel_var=T_rel*omegad_rel_var+Td_rel*omega_rel_var;
    etadd_rel=[etadd_rel;etadd_rel_var];
end


qddot=[vdc_i;etaddc_i;etadd_rel];












