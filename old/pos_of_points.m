%% Calculates the position of the point 'A' expressed in frame F1, knowing the position of the point in F2 and the relative position between F1 and F2

%%% REWRITE ARGUMENTS and RETURNS

%% Function Arguments:
%% rA_2= position of point 'A' w.r.t the frame F2
%% r2_1=location of the origin of frame F2 with respect to frame F1
%% eta2_1=eular angles of rotation of frame F2 with respect to the frame F1
%% Function Returns:
%% rA_1=the position of the point 'A' w.r.t the frame F1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rA_1=pos_of_points(r2_1,rA_2,eta2_1)

R2_1=Rot_tot(eta2_1);
rA_1=r2_1+R2_1*rA_2;

% function P=pos_of_points(p,rel_dist,eular_ang)
% 
% R=Rot_tot(eular_ang);
% P=p+R*rel_dist;









