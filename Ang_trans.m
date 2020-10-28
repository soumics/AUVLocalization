%% Calculate the Angular Transformation T(phi, theta) 
function [A]= Ang_trans(angles) %(phi->roll,theta->pitch,psi->yaw) [roll, pitch, yaw]

phi=angles(1); %roll
theta=angles(2); %pitch

    A=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);
        0 cos(phi) -sin(phi);
        0 sin(phi)/cos(theta) cos(phi)/cos(theta)];