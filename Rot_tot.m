%% Calculate the Total Rotation R((phi->roll,theta->pitch,psi->yaw)) 
function [A]= Rot_tot(angles)

%     R_x=Rot_x(angles(1));
%     R_y=Rot_y(angles(2));
%     R_z=Rot_z(angles(3));
%     
%     
%     
%     A=R_z*R_y*R_x;
    
    phi=angles(1);
    theta=angles(2);
    psi=angles(3);
      
    A=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
 cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)];