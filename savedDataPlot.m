clc;
close all;
clear all;

global no_of_links;

FEW_SAMPLES=0;
ERR_PLOT=0;
NORM_POSITION=1;
POSITION_PLOT=0;
POSITION_PLOT2=0;

STRAIGHTLINE=0;
CIRCLE=1;

Param;

NO_OF_SPARKERS=1;



if NO_OF_SPARKERS==1
    if CIRCLE==1
        XD1=[0;20;0];
        XD=XD1;
    end
    if STRAIGHTLINE==1
        XD1=[20;20;0];
        XD=XD1;
    end
end

if NO_OF_SPARKERS==2
    XD1=[20;20;0];
    XD2=[20;-20;0];
    XD=[XD1 XD2];
end

XD=[512716.72;4198376.63;0];

S1 = load('mr_data_5.mat');
S=load('currentWS.mat');
% S = load('auvstreamerDataCircOneSP.mat'); % Circle - One Sparker (static)
% S = load('auvstreamerDataCircTwoSP.mat'); % Circle - Two Sparkers (static)

% S = load('auvstreamerDataStrlnOneSP.mat'); % Straight Line - One Sparker (static)
% S = load('auvstreamerDataStrlnTwoSP.mat'); % Straight Line - Two Sparkers (static)

% S = load('auvstreamerDataCircMovOneSP.mat'); % Circle - One Sparker (moving)
% S = load('auvstreamerDataCircMovTwoSP.mat'); % Circle - Two Sparkers (moving)
%
% S = load('auvstreamerDataStrlnMovOneSP.mat'); % Straight Line - One Sparker
% S = load('auvstreamerDataStrlnMovTwoSP.mat'); % Straight Line - Two Sparkers
%
% S = load('auvstreamerDataStrlnOneSPSTOP.mat'); % Straight Line - One Sparker (stop midway)
% S = load('auvstreamerDataStrlnTwoSPSTOP.mat'); % Straight Line - Two Sparkers (stop midway)

%%%%%%%%%%%%%%%%%%% aquiring data from file %%%%%%%%%%%%%%%%%%%%%%%%%%%
q=S.q; qdot=S.qdot; qddot=S.qddot;
q_hat=S.q_hat; qdot_hat=S.qdot_hat; qddot_hat=S.qddot_hat;
pos_hyd_errnrm=S.pos_hyd_errnrm;
Xe=abs(S.X-S.Xhat);
length(S.t)
[row,col]=size(Xe)


rc_i0=[0;0;0]; % initial position of point C
etac_i0=[0*pi/2;0*pi/4;0*pi+0*pi/4]; % Eular angles of C w.r.t I [roll, pitch, yaw]



T=.1; % sampling time
%samples=1000;
R_circ=20.; % radius of the circle
v_cruise=0.7;% m/s
w_circ=+v_cruise/R_circ;
center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)];

%var_st_line=rc_i0;

samples=round(2*pi/norm(w_circ)/T) % number of samples
samples=1868
if STRAIGHTLINE==1
    samples=1000 % samples for straight line motion
end

% trajectory generation
for i=1:samples
    t(i)=i*T;
    % for Circle
    if CIRCLE==1
        pos_des_x(i)= center(1)+R_circ*cos(w_circ*t(i)-pi/2);
        pos_des_y(i)= center(2)+R_circ*sin(w_circ*t(i)-pi/2);
        pos_des_z(i)=rc_i0(3);
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=w_circ*t(i)+etac_i0(3);%pi
    end
    % for straight line
    if STRAIGHTLINE==1
        pos_des_x(i)=rc_i0(1)+t(i);
        pos_des_y(i)= rc_i0(2);
        pos_des_z(i)=rc_i0(3);
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=etac_i0(3)*0;%pi
    end
    
    
end

pos_des_x= S1.data(:,6);
pos_des_y= S1.data(:,5);
pos_des_z=S1.data(:,7);
roll_des=S1.data(:,7);
pitch_des=S1.data(:,7);
yaw_des=[];
for i=1:length(pos_des_x)-1
    yaw_ang_temp=atan2(pos_des_y(i+1)-pos_des_y(i),pos_des_x(i+1)-pos_des_x(i));
    yaw_des=[yaw_des;yaw_ang_temp];
end
yaw_des=[yaw_des;yaw_ang_temp];

%%%%%%%%%%%%%%%%%%%%%%% plot of few samples %%%%%%%%%%%%%%%%%%%%%%%%%
if FEW_SAMPLES==1
    cla
    hold on
    [row,col]=size(q);
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,1),qdot(:,1),qddot(:,1));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,1),qdot_hat(:,1),qddot_hat(:,1));
    %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/4)),qdot(:,round(col/4)),qddot(:,round(col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,round(col/4)),qdot_hat(:,round(col/4)),qddot_hat(:,round(col/4)));
    %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/2)),qdot(:,round(col/2)),qddot(:,round(col/2)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,round(col/2)),qdot_hat(:,round(col/2)),qddot_hat(:,round(col/2)));
    %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(3*col/4)),qdot(:,round(3*col/4)),qddot(:,round(3*col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,round(3*col/4)),qdot_hat(:,round(3*col/4)),qddot_hat(:,round(3*col/4)));
    %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
%     if CIRCLE==1
%         [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,col-25),qdot(:,col-25),qddot(:,col-25));
%         eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
%         drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
%         DrawPlot(P_draw,R_draw,0)
%         [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,col-25),qdot_hat(:,col-25),qddot_hat(:,col-25));
%         %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
%         %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
%         DrawPlot(P_draw_hat,R_draw_hat,1)
%     end
%     if STRAIGHTLINE==1
%         [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,col-1),qdot(:,col-1),qddot(:,col-1));
%         eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
%         drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
%         DrawPlot(P_draw,R_draw,0)
%         [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,col-1),qdot_hat(:,col-1),qddot_hat(:,col-1));
%         %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
%         %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
%         DrawPlot(P_draw_hat,R_draw_hat,1)
%     end
    
    plot3(XD(1,:),XD(2,:),XD(3,:),'*')
    plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
    
    %DrawPlot(r,R)
    
    
    
    % axis([-3 3 -3 3 -3 3])
    % axis([-2 2 -2 2 -2 2])
    %
    %  axis([-10 10 -10 10 -10 10])
    %
    axis square
    axis equal
%     if STRAIGHTLINE==1
%         axis([-15 samples*T -30 30 -10 15])
%     end
%     if CIRCLE==1
%         axis([-30 30 -10 50 -10 20])
%     end
    
    %        axis([-30 30 -30 30 -30 30])
    %  axis([-15 5 -5 5 0 5])
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    %title(sprintf('Time: %0.2f sec', t(i)));
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    %drawnow
    % view([0,90])
    %hold on;
    %if i==1
    %pause(.1)
end

%%%%%%%%%%%%% plot of the norm of error %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if NORM_POSITION==1
    
    figure
    plot(t(1:samples-8),pos_hyd_errnrm(:,1:samples-8))
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('error norm [m]','FontSize',12);
    str=[];
    for i=1:no_of_links
        str=[str,cellstr(char(strcat('Link ',num2str(i))))];
    end
    legend(str)
    set(gca,'fontsize',12)
end

%%%%%%%%%%%%%%% plot of the error %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ERR_PLOT==1
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples-25),Xe(1:3,1:samples-25))
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{r}}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples-25),Xe(row/2+1:row/2+1+2,1:samples-25))
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{v}}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    
    
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples-25),Xe(4:6,1:samples-25))
    legend(thisax,{'$\tilde{\phi}_c^i$','$\tilde{\theta}_c^i$','$\tilde{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\eta}_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples-25),Xe(row/2+1+3:row/2+1+5,1:samples-25))
    legend(thisax,{'$\dot{\tilde{\phi}}_c^i$','$\dot{\tilde{\theta}}_c^i$','$\dot{\tilde{\psi}}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\dot{\tilde{\eta}}_c^i \ [rad/s]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    
    
    if no_of_links>0
        for i=1:no_of_links
            figure
            thisax = subplot(2,1,1);
            
            plot(t(1:samples-25),Xe(7+3*(i-1):6+3*i,1:samples-25))
            legend(thisax,{['$\tilde{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\tilde{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            
            thisax = subplot(2,1,2);
            plot(t(1:samples-25),Xe((row/2+1+3*(i-1)):(row/2+3*i),1:samples-25))
            legend(thisax,{['$\dot{\tilde{\phi}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\tilde{\theta}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\tilde{\psi}}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\dot{\tilde{\eta}}_' num2str(i) '^' num2str(i-1) ' \ [rad/s]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            
        end
    end
end

%%%%%%%%% plot of the positions of all the points on the system %%%%%%%%%%%
if POSITION_PLOT==1
    
    hold on
    [row,col]=size(q);
    figure
    for k=1:200:samples
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,k),qdot(:,k),qddot(:,k));
        eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
        drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
        hold on
        DrawPlot(P_draw,R_draw,0)
        [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,k),qdot_hat(:,k),qddot_hat(:,k));
        
        DrawPlot(P_draw_hat,R_draw_hat,1)
    end
    plot3(XD(1,:),XD(2,:),XD(3,:),'*')
    
    plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
    
    %DrawPlot(r,R)
    %hold off
    
    
    % axis([-3 3 -3 3 -3 3])
    % axis([-2 2 -2 2 -2 2])
    %
    %  axis([-10 10 -10 10 -10 10])
    %
    axis square
    axis equal
%     if STRAIGHTLINE==1
%         axis([-15 samples*T -30 30 -10 10])
%     end
%     if CIRCLE==1
%         axis([-30 30 -10 50 -10 20])
%     end
    
    %        axis([-30 30 -30 30 -30 30])
    %  axis([-15 5 -5 5 0 5])
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    %title(sprintf('Time: %0.2f sec', t(i)));
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    
    
    
end

%%%%%%%%% plot of centroid and the last link %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if POSITION_PLOT2==1
    
    for i=1:samples-8
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        P_draw_X(i,:)=P_draw(1,:);
        P_draw_Y(i,:)=P_draw(2,:);
        P_draw_Z(i,:)=P_draw(3,:);
        
        [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,i),qdot_hat(:,i),qddot_hat(:,i));
        P_draw_hat_X(i,:)=P_draw_hat(1,:);
        P_draw_hat_Y(i,:)=P_draw_hat(2,:);
        P_draw_hat_Z(i,:)=P_draw_hat(3,:);
    end
    figure
    plot3(pos_des_x,pos_des_y,pos_des_z,'r')
    hold on
    
    plot3(P_draw_X(:,1),P_draw_Y(:,1),P_draw_Z(:,1))
    
    plot3(P_draw_hat_X(:,1),P_draw_hat_Y(:,1),P_draw_hat_Z(:,1),'k')
    
    plot3(P_draw_X(:,no_of_links+2),P_draw_Y(:,no_of_links+2),P_draw_Z(:,no_of_links+2),'g')
    
    plot3(P_draw_hat_X(:,no_of_links+2),P_draw_hat_Y(:,no_of_links+2),P_draw_hat_Z(:,no_of_links+2),'y')
    
%     for plt=1:no_of_links+2
%         plot3(P_draw_X(:,plt),P_draw_Y(:,plt),P_draw_Z(:,plt))
%         
%         plot3(P_draw_hat_X(:,plt),P_draw_hat_Y(:,plt),P_draw_hat_Z(:,plt),'k--')
%     end
    axis square
    axis equal
%     if STRAIGHTLINE==1
%         axis([-15 samples*T -30 30 -10 10])
%     end
%     if CIRCLE==1
%         axis([-30 30 -10 50 -10 20])
%     end
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    
    
%     figure
%     plot3(pos_des_x,pos_des_y,pos_des_z,'r')
%     hold on
%     
%     plot3(P_draw_X(:,no_of_links+2),P_draw_Y(:,no_of_links+2),P_draw_Z(:,no_of_links+2))
%     
%     plot3(P_draw_hat_X(:,no_of_links+2),P_draw_hat_Y(:,no_of_links+2),P_draw_hat_Z(:,no_of_links+2),'k--')
%     axis square
%     axis equal
%     if STRAIGHTLINE==1
%         axis([-15 samples*T -30 30 -10 10])
%     end
%     if CIRCLE==1
%         axis([-30 30 -10 50 -10 20])
%     end
%     set(gca, 'XDir', 'reverse')
%     set(gca, 'ZDir', 'reverse')
%     grid on
%     xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
%     ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
%     zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
end


