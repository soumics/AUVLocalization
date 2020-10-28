%% Calculate the transformation due to yaw R(psi) 
function [A]= Rot_z(psi)

A=[cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];