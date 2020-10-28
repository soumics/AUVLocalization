%% Calculate the transformation due to roll R(phi) 
function [A]= Rot_x(phi)

A=[1 0 0;
   0 cos(phi) -sin(phi);
   0 sin(phi) cos(phi)];