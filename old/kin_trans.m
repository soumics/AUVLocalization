%% This function calculates the derivative of the states given the states
%% and the control input, i.e., qdot=f(q,u)
function qdot=kin_trans(q,u)

global no_of_links;

%uc_c=u(1); vyc_c=u(2); wc_c=u(3); pic_c=u(4); qic_c=u(5); ric_c=u(6);


vc_c=u(1:3); % velocity of C in frame C
omega_ic_c=u(4:6); % angular velocity of C in frame C
omega_rel=u(7:6+3*no_of_links);

%%%%%%%%%%%%%%%%%% Kinematics of Point C %%%%%%%%%%%%%%%%%%%%%%%%%

eta_c=q(4:6); % Eular angles of C w.r.t I
Rc_i=Rot_tot(eta_c); % Rotation of C w.r.t I
Tc_i=Ang_trans(eta_c); % angular transformation of C w.r.t I

vc_i=Rc_i*vc_c; % velocity of C in I
etadc_i=Tc_i*omega_ic_c; % Eular rate of C in I

%%%%%%%%%%%%%%%%%% Kinematics of other points %%%%%%%%%%%%%%%%%%%%%%%%%

%init=7;
etad_rel=[];

for i=1:no_of_links%i=init:3:(init+3*no_of_links)-3
     eta_rel=q(6+3*(i-1)+1:6+3*i);
    T_rel=Ang_trans(eta_rel);
    %omega_rel(3*(i-1)+1:3*i)
    etad_rel=[etad_rel;T_rel*omega_rel(3*(i-1)+1:3*i)];
end


qdot=[vc_i;etadc_i;etad_rel];












