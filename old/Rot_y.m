%% Calculate the transformation due to pitch R(theta) 
function [A]= Rot_y(theta)

A=[cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];