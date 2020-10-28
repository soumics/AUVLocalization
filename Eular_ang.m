%% Calculate the Eular Angles (phi->roll,theta->pitch,psi->yaw) from a rotation matrix R 
function [A]= Eular_ang(R)

phi=atan2(R(3,2),R(3,3));
%theta=atan2(-R(3,1),norm([R(3,2) R(3,3)]));
%theta=-atan(R(3,1)/sqrt(1-R(3,1)^2));
theta=asin(-R(3,1));%/sqrt(1-R(3,1)^2));
psi=atan2(R(2,1),R(1,1));
A=[phi;theta;psi];