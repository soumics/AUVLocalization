%%REWRITE THE DESCRIPTION OF THE FUNCTION

%% This function returns the origin and rotation [w.r.t inertial frame] of frame F2
%% with the Function Arguments given below
%% Function Arguments:
%% r2_1=Position ofthe origin of the frame F2 with respect to the frame F1
%% r32_3=Vector from the frame F3 to the frame F2 expressed in F3
%% eta2_1=Eular angles of the frame F2 with respect to the frame F1
%% eta3_2=Relative Eular angles of the frame F3 with respect to the frame F2

%% Functions Returns:
%% r3_1=Position of point 3 with respect to the frame 1
%% R3_1=Rotation matrix of the frame 3 with respect to the frame 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [r3_1,R3_1]=pos_rot(r2_1,r32_3,eta2_1,eta3_2)
R3_2=Rot_tot(eta3_2);
R2_1=Rot_tot(eta2_1);
r23_2=-R3_2*r32_3;

%r3_1=pos_of_points(r2_1,r23_2,eta2_1);
r3_1=r2_1+R2_1*r23_2;

R2_1=Rot_tot(eta2_1);
R3_1=R2_1*R3_2;


