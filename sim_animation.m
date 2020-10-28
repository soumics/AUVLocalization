clc;
close all;
clear all;

% S = load('auvstreamerDataCircOneSP.mat'); % Circle - One Sparker (static)
S = load('auvstreamerDataCircTwoSP.mat'); % Circle - Two Sparkers (static)

% S = load('auvstreamerDataStrlnOneSP.mat'); % Straight Line - One Sparker (static)
% S = load('auvstreamerDataStrlnTwoSP.mat'); % Straight Line - Two Sparkers (static)

% S = load('auvstreamerDataCircMovOneSP.mat'); % Circle - One Sparker (moving)
% S = load('auvstreamerDataCircMovTwoSP.mat'); % Circle - Two Sparkers (moving)
%
% S = load('auvstreamerDataStrlnMovOneSP.mat'); % Straight Line - One Sparker
% S = load('auvstreamerDataStrlnMovTwoSP.mat'); % Straight Line - Two Sparkers
%
% S = load('auvstreamerDataStrlnOneSPSTOP.mat'); % Straight Line - One Sparker (stop midway)
% S = load('auvstreamerDataStrlnTwoSPSTOP.mat'); % Straight Line - Two
% Sparkers (stop midway)

NORM_POSITION=0;
SAVE_FIG=0;
ANIMATION=0;
ERR_PLOT=1;

NO_OF_SPARKERS=2;

Param

rc_i0=[0;0;1]; % initial position of point C
etac_i0=[0*pi/2;0*pi/4;0*pi+pi/4];

if NO_OF_SPARKERS==1
    XD1=[10;20;0];
    XD=XD1;
end

if NO_OF_SPARKERS==2
    XD1=[10;20;0];
    XD2=[10;-20;0];
    XD=[XD1 XD2];
end

q=S.q;
qdot=S.qdot;
qddot=S.qddot;
q_hat=S.q_hat;
qdot_hat=S.qdot_hat;
qddot_hat=S.qddot_hat;
t=S.t;
pos_hyd_errnrm=S.pos_hyd_errnrm;

T=.1;
%samples=1000;
R_circ=20.;
v_cruise=0.7;% m/s
w_circ=+v_cruise/R_circ;
center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)];
samples=round(2*pi/norm(w_circ)/T)

for i=1:samples
    t(i)=i*T;
    pos_des_x(i)= center(1)+R_circ*cos(w_circ*t(i)-pi/2);
    pos_des_y(i)= center(2)+R_circ*sin(w_circ*t(i)-pi/2);
    pos_des_z(i)=rc_i0(3);
    roll_des(i)=etac_i0(1)*0;
    pitch_des(i)=etac_i0(2)*0;
    yaw_des(i)=w_circ*t(i)+etac_i0(3);
end
samples=500;
for i=1:samples
    if ANIMATION==1
        cla
        hold on
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
        %drawEllipsoid([r(:,1)' 1 .5 .5 eularAng(3) eularAng(2) eularAng(1)]);
        drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
        
        [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,i),qdot_hat(:,i),qddot_hat(:,i));
        
        
        DrawPlot(P_draw,R_draw,0)
        
        DrawPlot(P_draw_hat,R_draw_hat,1)
        
        
        plot3(XD(1,:),XD(2,:),XD(3,:),'*')
        
        plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
        
        axis([-10 10 -10 2 0 10])
        % axis([-50 50 -50 50 -50 50])
        %
        %axis square
        %         axis equal
        %         axis([-50 50 -50 50 -50 50])
        
        %        axis([-30 30 -30 30 -30 30])
        %  axis([-15 5 -5 5 0 5])
        set(gca, 'XDir', 'reverse')
        set(gca, 'ZDir', 'reverse')
        title(sprintf('t: %0.2f sec', t(i)),'fontsize',12);
        set(gca,'fontsize',12)
        grid on
        
        pause(.1)
    end
    
end

if NORM_POSITION==1
    
    figure
    plot(t(1:samples),pos_hyd_errnrm(:,1:samples))
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('error norm [m]','FontSize',12);
    str=[];
    for i=1:no_of_links
        str=[str,cellstr(char(strcat('Link ',num2str(i))))];
    end
    legend(str)
    set(gca,'fontsize',12)
    
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'errnorm.jpg')
        saveas(gcf,'errnorm','epsc')
        cd ..
    end
end

if ERR_PLOT==1
    
    %% error plot
    
    Xe=abs(S.X-S.Xhat);
    [row,col]=size(Xe);
    col=samples+1
    length(t)
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xe(1:3,1:col-1),'LineWidth',2)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{r}}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xe(row/2+1:row/2+1+2,1:col-1),'LineWidth',2)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{v}}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'err_auv_pos_vel.jpg')
        saveas(gcf,'err_auv_pos_vel','epsc')
        cd ..
    end
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xe(4:6,1:col-1),'LineWidth',2)
    legend(thisax,{'$\tilde{\phi}_c^i$','$\tilde{\theta}_c^i$','$\tilde{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\eta}_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xe(row/2+1+3:row/2+1+5,1:col-1),'LineWidth',2)
    legend(thisax,{'$\dot{\tilde{\phi}}_c^i$','$\dot{\tilde{\theta}}_c^i$','$\dot{\tilde{\psi}}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\dot{\tilde{\eta}}_c^i \ [rad/s]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'err_auv_ang_angr.jpg')
        saveas(gcf,'err_auv_ang_angr','epsc')
        cd ..
    end
    
    if no_of_links>0
        for i=1:no_of_links
            figure
            thisax = subplot(2,1,1);
            
            plot(t(1:samples),Xe(7+3*(i-1):6+3*i,1:col-1),'LineWidth',2)
            legend(thisax,{['$\tilde{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\tilde{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            
            thisax = subplot(2,1,2);
            plot(t(1:samples),Xe((row/2+1+3*(i-1)):(row/2+3*i),1:col-1),'LineWidth',2)
            legend(thisax,{['$\dot{\tilde{\phi}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\tilde{\theta}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\tilde{\psi}}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\dot{\tilde{\eta}}_' num2str(i) '^' num2str(i-1) ' \ [rad/s]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            if SAVE_FIG==1
                cd img
                % saveas(gcf,['err_Link' num2str(i) '_ang_angr.jpg'])
                saveas(gcf,['err_Link' num2str(i) '_ang_angr'],'epsc')
                cd ..
            end
        end
    end
end