clc;
close all;
clear all;
% hold on;
%digits(64)

ANIMATION=0;
var_bool=1;
Param_symb

q= sym('q',[6+3*no_of_links 1],'real')
qdot= sym('qdot',[6+3*no_of_links 1],'real')
qddot= sym('qddot',[6+3*no_of_links 1],'real')
tau= sym('tau',[6+3*no_of_links 1],'real')


qddot_links=kron(ones(no_of_links,1),[0;0;0]); 
qddot_auv=zeros(6,1);
qddot=[qddot_auv;qddot_links];

Tau_bar_temp=inv_dyn(q,qdot,qddot,0);
[row,col]=size(Tau_bar_temp);
Tau_bar=[];
for ii=1:col
    Tau_bar=[Tau_bar;Tau_bar_temp(:,ii)];
end

% Setting g, and velocity of the states to ZERO [vector]
g=[0;0;0];

% Calculation of the columns of the mass matrix

for i=1:6+3*no_of_links
    qdot_temp=zeros(6+3*no_of_links,1);    
    qddot_temp=zeros(6+3*no_of_links,1);
    qddot_temp(i)=1;
    
    M_curr_temp=inv_dyn(q,qdot_temp,qddot_temp,0);
    [row1,col1]=size(M_curr_temp);
    
    M_temp=[];
    for j=1:col1
        M_temp=[M_temp;M_curr_temp(:,j)];
    end
    M(:,i)=M_temp;
end

% restoring g to its original value
g=[0;0;g0];

% Calculation of the acceleration with mass matrix, Tau_bar(coriolis force
% etc. ), and given Tau (input)
M
Tau_bar
simplify(M)
simplify(Tau_bar)

%qddot=inv(M)*(Tau-Tau_bar);



% rc_i0=[0;0;1]; % initial position of point C
% 
% etac_i0=[pi/2;pi/4;0*pi+pi/4]; % Eular angles of C w.r.t I [roll, pitch, yaw]
% 
% eta_rel0=kron(ones(no_of_links,1),[0;0;0]);
% %eta_rel0(1)=0
% %eta_rel0(2)=pi/3
% %eta_rel0(3)=-pi/100
% %eta_rel0(4)=-pi/4
% vc_i0=[0;0;0]; % initial velocity of point C
% 
% etadc_i0=[0;0;0]; % Eular rate of C w.r.t I
% 
% etad_rel0=kron(ones(no_of_links,1),[0;0;0]); 
% 
% 
% fx=0.0; fy=0; fz=0; taux=0.0; tauy=0; tauz=0; % forces and torques on auv
% 
% tau_rel_link=[0;0;0];
% tau_rel=kron(ones(no_of_links,1),tau_rel_link); % torques on links
% %tau_rel(3*no_of_links-1)=1
% % tau_rel(3)=0;
% % tau_rel(6)=pi/50;
% %Tau_auv=[Rot_tot(etadc_i0)*[fx;fy;fz];Rot_tot(etadc_i0)*[taux;tauy;tauz]];
% Tau_auv=[[fx;fy;fz];[taux;tauy;tauz]];
% Tau=[Tau_auv;tau_rel];
% 
% NO_ROLL=0;
% NO_YAW=0;
% NO_CONTROL=0;
% q0=[rc_i0;etac_i0;eta_rel0];
% qd0=[vc_i0;etadc_i0;etad_rel0];
% qdd0=zeros(6+3*no_of_links,1);
% q=q0; qdot=qd0; qddot=qdd0;
% 
% T=.1;
% %samples=1000;
% R_circ=20.;
%     v_cruise=0.7;% m/s
%     w_circ=+v_cruise/R_circ;
%     center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)];
%     samples=round(2*pi/norm(w_circ)/T)
% %samples=100
% samples=2
% for i=1:samples
%     t(i)=i*T;
%     
%     pos_des_x(i)= center(1)+R_circ*cos(w_circ*t(i)-pi/2);
%     pos_des_y(i)= center(2)+R_circ*sin(w_circ*t(i)-pi/2);
%     pos_des_z(i)=rc_i0(3);
%     roll_des(i)=etac_i0(1)*0;
%     pitch_des(i)=etac_i0(2)*0;
%     yaw_des(i)=w_circ*t(i)+etac_i0(3);%pi
%     
%    %i
%    if NO_ROLL==1
%    for j=no_of_links-1:-1:0
%      q(7+3*j,i)=0;
%      qdot(7+3*j,i)=0;
%    end
%    end
%    % roll_des(i)
%    % q(4,i)
%    % qdot(4,i)
%     taux=.02*(roll_des(i)-q(4,i))-0.0*qdot(4,i);
%    % pitch_des(i)
%    % q(5,i)
%    % qdot(5,i)
%     tauy=5*(pitch_des(i)-q(5,i))-20*qdot(5,i);
%    % yaw_des(i)
%    % q(6,i)
%    % qdot(6,i)
%     tauz=5*(yaw_des(i)-q(6,i))-20*qdot(6,i);
%     
%     fx_i=10*(pos_des_x(i)-q(1,i));%-3*qdot(1,i)
%     fy_i=10*(pos_des_y(i)-q(2,i));%-3*qdot(2,i)
%     fz_i=10*(pos_des_z(i)-q(3,i));%-3*qdot(3,i)
%     f_temp=Rot_tot(q(4:6,i))'*[fx_i;fy_i;fz_i];
%     fx=f_temp(1);
%     fy=f_temp(2);
%     fz=f_temp(3);
%    % [fx;fy;fz]=Rot_tot(q(4:6,i))'*[fx_i;fy_i;fz_i]
%   % T_rpy(q(4:6,i))*[taux;tauy;tauz]
%   % T_rpy(q(4:6,i))'*[taux;tauy;tauz]
%     Tau_sol=[taux;tauy;tauz];
%    % Tau_auv=[Rot_tot(q(4:6,i))*[fx;fy;fz];Rot_tot(q(4:6,i))*[taux;tauy;tauz]];%*0+Rot_tot(q(4:6,i))*[0;0;1]]
%     
%    %Tau_auv=[[fx_i;fy_i;fz_i];Rot_tot(q(4:6,i))*[taux;tauy;tauz]];%*0+Rot_tot(q(4:6,i))*[0;0;1]]
%     Tau_auv=[[fx_i;fy_i;fz_i];Tau_sol];%*0+Rot_tot(q(4:6,i))*[0;0;1]]
%      %   T_rpy(q(4:6,i))*[taux;tauy;tauz]]
%     if NO_CONTROL==1
%         Tau_auv=0*Tau_auv;
%     end
%     
%     %Tau_auv=[Rot_tot(q(4:6,i))*[fx;fy;fz];Rot_tot(q(4:6,i))*[taux;tauy;tauz]]
%     Tau=[Tau_auv;tau_rel];
%     
% %     if t(i)>10
% %       Tau=0*Tau;  
% %     end
%     
% %     if i==50
% %         break;
% %     end
%      %if t<=100
% %         ud(5)=pi/10;
% %     end
% %     if t>100
% %         ud(5)=0;
% %         ud(6)=pi/10;
% %     end
%     
%     qddot(:,i)=dir_dyn(q(:,i),qdot(:,i),Tau);
%     if NO_ROLL==1
%         for j=no_of_links-1:-1:0
%      q(7+3*j,i)=0;
%      qdot(7+3*j,i)=0;
%      qddot(7+3*j,i)=0;
%         end
%     end
%     %disp('tau_sol') 
%     %Tau_sol'
%     %disp('tau_i') 
%      %Tau'
%        
%     %disp('qddot') 
%     %qddot(:,i)'
%      
%     qdot(:,i+1)=qddot(:,i)*T+qdot(:,i); 
%     %disp('qdot')  
%     %qdot(:,i+1)'
%     
%     q(:,i+1)=q(:,i)+qdot(:,i+1)*T+0*qddot(:,i)*T^2/2;
%     %disp('q') 
%     %q(:,i+1)'
%     
% %     for j=no_of_links-1:no_of_links-1
% %     q(7+3*j,i+1)=0;
% %     qdot(7+3*j,i+1)=0;
% %     end
%     
%     [r,R]=pos_rot_pnts(q(1:6+3*no_of_links,i));
%     if ANIMATION==1
%         cla
%         hold on
%         [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
%         
%         DrawPlot(P_draw,R_draw)
%         DrawPlot(r,R)
%         hold off
%         
%         
%        % axis([-3 3 -3 3 -3 3])
%        % axis([-2 2 -2 2 -2 2])
%         %
%       %  axis([-10 10 -10 10 -10 10])
%        %  
%         %axis square
%         axis equal
%           axis([-50 50 -50 50 -50 50])
%       
% %        axis([-30 30 -30 30 -30 30])
%       %  axis([-15 5 -5 5 0 5])
%         set(gca, 'XDir', 'reverse')
%         set(gca, 'ZDir', 'reverse')
%         title(sprintf('Time: %0.2f sec', t(i)));
%         grid on
%         %drawnow
%         % view([0,90])
%         %hold on;
%         %if i==1
%         pause(.1)
%         %end
% %         if i==20
% %             break;
% %         end
%     end
%     
% end
% 
% 
% 
% if var_bool==1
%     plotofdata(q,qdot,qddot,samples,t);
% end
