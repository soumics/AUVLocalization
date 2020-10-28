%%%%%%%%%%%%% simulation of the system of hydrophones lowed with auv %%%%%%%
clc;
close all;
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% configuration variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ANIMATION=0; % 1 to watch animation else 0
var_bool=0; % 1 to see position, velocity, acceleration, force w.r.t all frames else 0
ACT_DATA_PLOT=0; % 1 to see actual data plot else 0

SAVE_FIG=0; % 1 to save figures else 0

CIRCLE=1; % 1 to extract results for circular trajectory else 0
STRAIGHTLINE=0; % 1 to extract results for straight line trajectory else 0
FEW_SAMPLE_PLOT=0; % 1 to plot a few samples else 0
POSITION_PLOT=1; % 1 to see plot of the positions of all joints and auv

NO_ROLL=0; % 0 for no roll else 1
NO_YAW=0; % 0 for no yaw else 1
NO_CONTROL=0; % 0 for no control else 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Param % invoking all parameters from Param.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Initial configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rc_i0=[0;0;0]; % initial position of point C

etac_i0=[0*pi/2;0*pi/4;0*pi+0*pi/4]; % Eular angles of C w.r.t I [roll, pitch, yaw]

eta_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler angles between two consecutive frmes
%eta_rel0(2)=pi/2-0.1;
% eta_rel0(3)=-pi/18;
% eta_rel0(3)=-pi/100
% eta_rel0(4)=-pi/4
vc_i0=[0;0;0]; % initial velocity of point C

etadc_i0=[0;0;0]; % Eular rate of C w.r.t I

etad_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler rates between two consecutive frmes


fx=0.0; fy=0; fz=0; taux=0.0; tauy=0; tauz=0; % forces and torques on auv

tau_rel_link=[0;0;0];
tau_rel=kron(ones(no_of_links,1),tau_rel_link); % torques on links
%tau_rel(3*no_of_links-1)=1
% tau_rel(3)=0;
% tau_rel(6)=pi/50;
%Tau_auv=[Rot_tot(etadc_i0)*[fx;fy;fz];Rot_tot(etadc_i0)*[taux;tauy;tauz]];
Tau_auv=[[fx;fy;fz];[taux;tauy;tauz]]; % forces and torques on auv
Tau=[Tau_auv;tau_rel]; % forces and torques on auv and the links

%%%% state initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0=[rc_i0;etac_i0;eta_rel0];
qd0=[vc_i0;etadc_i0;etad_rel0];
qdd0=zeros(6+3*no_of_links,1);
q=q0;
qdot=qd0;
qddot=qdd0;

X0=[q;qdot];
X=X0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=.1; % sampling interval
%samples=1000;
R_circ=20.;
v_cruise=0.7;% m/s
w_circ=+v_cruise/R_circ;
center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)];
if CIRCLE==1
    samples=round(2*pi/norm(w_circ)/T)
    samples=2000
end
if STRAIGHTLINE==1
    samples=1000
end
samples=200
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% trajectory generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:samples
    t(i)=i*T;
    if CIRCLE==1
        pos_des_x(i)= center(1)+R_circ*cos(w_circ*t(i)-pi/2);
        pos_des_y(i)= center(2)+R_circ*sin(w_circ*t(i)-pi/2);
        pos_des_z(i)=rc_i0(3);
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=w_circ*t(i)+etac_i0(3);%pi
    end
    
    if STRAIGHTLINE==1
        pos_des_x(i)=rc_i0(1)+t(i);
        pos_des_y(i)= rc_i0(2);
        pos_des_z(i)=rc_i0(3)-0;
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=etac_i0(3)*0;
    end
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% motion planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:samples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% control input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            q(7+3*j,i)=0;
            qdot(7+3*j,i)=0;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%% designed controller (auv)   %%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%% angular motion control %%%%%%%%%%%%%%%%%%%%%%%%
    
    taux=0.02*(roll_des(i)-q(4,i))-0.0*qdot(4,i);  % roll control
    
    tauy=5*(pitch_des(i)-q(5,i))-20*qdot(5,i); % pitch control
    
    tauz=5*(yaw_des(i)-q(6,i))-20*qdot(6,i); % yaw control
    
    %%%%%%%%%%%% translational motion control %%%%%%%%%%%%%%%%%%%%
    fx_i=20*(pos_des_x(i)-q(1,i))-0*qdot(1,i); % x control
    fy_i=20*(pos_des_y(i)-q(2,i))-0*qdot(2,i); % y control
    fz_i=20*(pos_des_z(i)-q(3,i))-0*qdot(3,i); % z control
    
    Tau_sol=[taux;tauy;tauz]; % control torque
    
    Tau_auv=[[fx_i;fy_i;fz_i];Tau_sol]; % force and torque control
    
     %%%%%%%%%%%%%% no control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if NO_CONTROL==1
        
        Tau_auv=[10;0;0;0;0;0];
        % Tau_auv=0*Tau_auv;
    end
    
   
    Tau=[Tau_auv;tau_rel];
    % total torque of auv and the links
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% next state calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    qddot(:,i+1)=dir_dyn(q(:,i),qdot(:,i),Tau); % acceleration
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            q(7+3*j,i)=0;
            qdot(7+3*j,i)=0;
            qddot(7+3*j,i)=0;
        end
    end
    
    
    qdot(:,i+1)=qddot(:,i+1)*T+qdot(:,i); % velocity
    
    q(:,i+1)=q(:,i)+qdot(:,i+1)*T+0*qddot(:,i+1)*T^2/2; % position
    
    X(:,i+1)=[q(:,i+1);qdot(:,i+1)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%% animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ANIMATION==1
        cla
        hold on
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        
        DrawPlot(P_draw,R_draw,0)
       
        hold off
        
        
        axis equal
        axis([-10 100 -50 50 -50 50])
        
      
        set(gca, 'XDir', 'reverse')
        set(gca, 'ZDir', 'reverse')
        title(sprintf('Time: %0.2f sec', t(i)));
        grid on
        
        pause(.1)
        
    end
    i
end
%%%%%%%%%%%%%%%%%%%%%%%%% few sample plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if FEW_SAMPLE_PLOT==1
    cla
    hold on
    [row,col]=size(q);
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,1),qdot(:,1),qddot(:,1));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/4)),qdot(:,round(col/4)),qddot(:,round(col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/2)),qdot(:,round(col/2)),qddot(:,round(col/2)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(3*col/4)),qdot(:,round(3*col/4)),qddot(:,round(3*col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,col-25),qdot(:,col-25),qddot(:,col-25));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    
    plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
    
    
    hold off
    
    
   
    axis equal
    if STRAIGHTLINE==1
        axis([-15 samples*T -2 2 -0 15])
    end
    if CIRCLE==1
        axis([-20 20 -10 50 -10 10])
    end
    
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    
end

%%%%%% plot of positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if POSITION_PLOT==1
    
    for i=1:samples
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        P_draw_X(i,:)=P_draw(1,:);
        P_draw_Y(i,:)=P_draw(2,:);
        P_draw_Z(i,:)=P_draw(3,:);
    end
    figure
    plot(t(1:samples),P_draw_X)
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',12);
    ylabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    str=[];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Cx}^i$')))];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Px}^i$')))];
    for i=1:no_of_links
        str=[str,cellstr(char(strcat('$\mathbf{r}_{P',num2str(i),'x}^i$')))];
    end
    legend(str,'Interpreter','LaTex','FontSize',12);
    set(gca,'fontsize',12)
    figure
    plot(t(1:samples),P_draw_Y)
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    str=[];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Cy}^i$')))];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Py}^i$')))];
    for i=1:no_of_links
        str=[str,cellstr(char(strcat('$\mathbf{r}_{P',num2str(i),'y}^i$')))];
    end
    legend(str,'Interpreter','LaTex','FontSize',12);
    set(gca,'fontsize',12)
    figure
    plot(t(1:samples),P_draw_Z,t(1:samples),pos_des_z,'--')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    str=[];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Cz}^i$')))];
    str=[str,cellstr(char(strcat('$\mathbf{r}_{Pz}^i$')))];
    for i=1:no_of_links
        str=[str,cellstr(char(strcat('$\mathbf{r}_{P',num2str(i),'z}^i$')))];
    end
    legend(str,'Interpreter','LaTex','FontSize',12);
    set(gca,'fontsize',12)
    [row,col]=size(P_draw_X);
    figure
    h=plot(pos_des_x,pos_des_y,'g',P_draw_X(1,1),P_draw_Y(1,1),'k>',P_draw_X(1,no_of_links+2),P_draw_Y(1,no_of_links+2),'k>',...
        P_draw_X(:,1),P_draw_Y(:,1),'r',P_draw_X(:,no_of_links+2),P_draw_Y(:,no_of_links+2),'b',...
        P_draw_X(row,1),P_draw_Y(row,1),'ko',P_draw_X(row,no_of_links+2),P_draw_Y(row,no_of_links+2),'ko');
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    legend(h([1 4 5]),'Desired Trajectory','Centroid Trajectory','Last Link Trajectory','FontSize',12)
    set(gca,'fontsize',12)
end

%%%%%%%%%%%%%%%%%%%%% plot of actual data %%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ACT_DATA_PLOT==1
    %% plot of actual data
    
    
    [row,col]=size(X);
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),X(1:3,1:col-1),'LineWidth',2)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\mathbf{r}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),X(row/2+1:row/2+1+2,1:col-1),'LineWidth',2)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\mathbf{v}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),X(4:6,1:col-1),'LineWidth',2)
    legend(thisax,{'$\phi_c^i$','$\theta_c^i$','$\psi_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\eta_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),X(row/2+1+3:row/2+1+5,1:col-1),'LineWidth',2)
    legend(thisax,{'$\dot{\phi}_c^i$','$\dot{\theta}_c^i$','$\dot{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\dot{\eta}_c^i \ [rad/s]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    if no_of_links>0
        for i=1:no_of_links
            figure
            thisax = subplot(2,1,1);
            
            plot(t(1:samples),X(7+3*(i-1):6+3*i,1:col-1),'LineWidth',2)
            legend(thisax,{['$\phi_' num2str(i) '^' num2str(i-1) '$'],['$\theta_' num2str(i) '^' num2str(i-1) '$'],['$\psi_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\tilde{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            thisax = subplot(2,1,2);
            plot(t(1:samples),X((row/2+1+3*(i-1)):(row/2+3*i),1:col-1),'LineWidth',2)
            legend(thisax,{['$\dot{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\dot{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad/s]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





if var_bool==1
    plotofdata(q,qdot,qddot,samples,t);
end
