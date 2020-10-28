function T =T_rpy(alpha)% T_rpy(roll,pitch,yaw)

psi=alpha(1); %roll
theta=alpha(2); %pitch
phi=alpha(3); %yaw

T=[cos(phi)*cos(theta) -sin(phi) 0;
    sin(phi)*cos(theta) cos(phi) 0;
    -sin(theta) 0 1];
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


end

