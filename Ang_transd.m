%% Calculate the derivative Angular Transformation dotT(phi,theta,phid,thetad) 
function [A]= Ang_transd(ang_dang)

phi=ang_dang(1);
theta=ang_dang(2);
% psi=ang_dang(3);
phid=ang_dang(4);
thetad=ang_dang(5);
% psid=ang_dang(6);

e11=0;
e12=cos(phi)*phid*tan(theta)+sin(phi)*sec(theta)^2*thetad;
e13=cos(phi)*sec(theta)^2*thetad-sin(phi)*phid*tan(theta);
e21=0;
e22=-sin(phi)*phid;
e23=-cos(phi)*phid;
e31=0;
e32=(cos(phi)/cos(theta))*phid+(sin(phi)*sin(theta)/cos(theta)^2)*thetad;
e33=(cos(phi)*sin(theta)/cos(theta)^2)*thetad-(sin(phi)/cos(theta))*phid;
A=[e11 e12 e13;
    e21 e22 e23;
    e31 e32 e33];