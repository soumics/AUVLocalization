%% Calculate the Angular Transformation T(phi, theta) 
function [A]= Ang_trans_inv(angles) %(phi->roll,theta->pitch,psi->yaw) [roll, pitch, yaw]

phi=angles(1); %roll
theta=angles(2); %pitch

    A=[1 0 -sin(theta);
        0 cos(phi) cos(theta)*sin(phi);
        0 -sin(phi) cos(phi)*cos(theta)];