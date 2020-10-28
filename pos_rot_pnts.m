%% This function calculates the positions and rotation matrices of all the
%% origins of the frames including the centroid's and the point of
%% attachment's
%% Function Arguments:
%% q=states (auv position, orientation, and relative eular angles)
%% Function Returns:
%% P=Position of all points including the 1st point
%% R=Rotation matrices of all points including the 1st point

function [p,R]=pos_rot_pnts(q)

global rpipi_1_pi;
global r_PC_p;
global etap_c;
global no_of_links;


eta_rel=[];
for j=1:no_of_links
    eta_rel=[eta_rel q(6+3*(j-1)+1:6+3*j)];
end

%%%%%%%% Position and Rotation of the Point c %%%%%%%%%%%%%%%%%%%%%%%%%%%%
rC_i=q(1:3);
etac_i=q(4:6);
Rc_i=Rot_tot(etac_i);

% T_c = [Rc_i rC_i; 0 0 0 1];
% 
% %hold on
% DrawFrame(T_c,1);
% hold off

%%%%%%%% Position and Rotation of the Point P %%%%%%%%%%%%%%%%%%%%%%%%%%%%
[rP_i,Rp_i]=pos_rot(rC_i,r_PC_p,etac_i,etap_c);
% T_p = [Rp_i rP_i; 0 0 0 1];
% 
% hold on
% DrawFrame(T_p,1);
% hold off


%%%%%%%% Position and Rotation of the other points %%%%%%%%%%%%%%%%%%%%%%%%
p_var=rP_i;
eta_var=Eular_ang(Rp_i);
p=[rC_i p_var];
R(:,:,1)=Rc_i;
R(:,:,2)=Rp_i;
for i=1:no_of_links
    eta_rel_var=eta_rel(:,i);
    r_rel_var=rpipi_1_pi(:,i);
    [p_nxt,R_nxt]=pos_rot(p_var,r_rel_var,eta_var,eta_rel_var);
%     T_var = [R_nxt p_nxt; 0 0 0 1];
%     hold on
%     DrawFrame(T_var,1);
%     hold off
    p_var=p_nxt;
    eta_var=Eular_ang(R_nxt);
    p=[p p_nxt];
    R(:,:,2+i)=R_nxt;
end
