clc;
close all;
clear all;

ANIMATION=0; % 1 to watch animation else 0
var_bool=0; % 1 to see position, velocity, acceleration, force w.r.t all frames else 0
ERR_PLOT=0; % 1 to see error plot else 0
EIG_PLOT=0; % 1 to see eigen value plot else 0
ACT_DATA_PLOT=1; % 1 to see actual data plot else 0
NORM_POSITION=0; % 1 to see the norm position else 0

FEW_SAMPLES=0; % 1 to plot a few samples else 0

CIRCLE=0; % 1 to extract results for circular trajectory else 0
STRAIGHTLINE=1; % 1 to extract results for straight line trajectory else 0

MOVING_SPARKER=0; % 1 to extract results for moving saprkers else 0

STOP_MIDWAY=0; % 1 to extract results when the auv stops in the midway else 0

SAVE_FIG=0; % 1 to save figures else 0
SAVE_FILE=0; % 1 to save to file else 0
NO_OF_SPARKERS=2; % no of sparkers 1 0r 2

NO_ROLL=0; % 0 for no roll else 1
NO_YAW=0; % 0 for no yaw else 1
NO_CONTROL=0; % 0 for no control else 1
NO_EKF_corr=1; % 0 for no EKF correction else 1, ie, only prediction

Param % invoking all parameters from Param.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sparker positions depending on number of sparkers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if NO_OF_SPARKERS==1
    XD1=[20;20;0];
    XD=XD1;
end

if NO_OF_SPARKERS==2
    XD1=[20;20;0];
    XD2=[20;-20;0];
    XD=[XD1 XD2];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% initial configuration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rc_i0=[0;0;0]; % initial position of point C

etac_i0=[0*pi/2;0*pi/4;0*pi+0*pi/4]; % Euler angles of C w.r.t I [roll, pitch, yaw]

eta_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler angles between two consecutive frmes
%eta_rel0(1)=0
%eta_rel0(2)=pi/3
%eta_rel0(3)=-pi/100
%eta_rel0(4)=-pi/4
vc_i0=[0;0;0]; % initial velocity of point C

etadc_i0=[0;0;0]; % Eular rate of C w.r.t I

etad_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler rates between two consecutive frmes


fx=0.0; fy=0; fz=0; taux=0.0; tauy=0; tauz=0; % forces and torques on auv

tau_rel_link=[0;0;0];
tau_rel=kron(ones(no_of_links,1),tau_rel_link); % torques on links

Tau_auv=[[fx;fy;fz];[taux;tauy;tauz]]; % forces and torques on AUV
Tau=[Tau_auv;tau_rel]; % forces and torques on auv and the links

%%%% state initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0=[rc_i0;etac_i0;eta_rel0];
qd0=[vc_i0;etadc_i0;etad_rel0];
qdd0=zeros(6+3*no_of_links,1);
q=q0; qdot=qd0; qddot=qdd0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Process covariance initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%P_cov=0.1*eye(2*(6+3*no_of_links)); % Process covariance initialization
P_cov=blkdiag(0.1*eye(3),.0012*eye(3),.0012*eye(3*no_of_links),0.1*eye(3),.0012*eye(3),.0012*eye(3*no_of_links));
%P_cov=eye(2*(6+3*no_of_links));

for p=1:no_of_links
    P_cov(4+3*p,4+3*p)=0;
    P_cov(10+3*no_of_links+3*p,10+3*no_of_links+3*p)=0;
end
%P_cov=0*P_cov;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
eig_cov=eig(P_cov); % eigen value of process covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% process error covariance initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Q=blkdiag(0.001*eye(3),.01*(2/180)*pi*eye(3),.001*(2/180)*pi*eye(3*no_of_links),0.001*eye(3),.001*(2/180)*pi*eye(3),.001*(2/180)*pi*eye(3*no_of_links));
Q=blkdiag(0.01*eye(3),.0012*eye(3),.0012*eye(3*no_of_links),0.01*eye(3),.0012*eye(3),.0012*eye(3*no_of_links));

for p=1:no_of_links
    Q(4+3*p,4+3*p)=0;
    Q(6+3*no_of_links+4+3*p,10+3*no_of_links+3*p)=0;
end
%Q=0*Q;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X0=[q;qdot]; % initial state vector
X=X0;

qddot_hat=qddot;

%X0(8)=pi/3;
T=.1; % sampling interval

Xhat0=X0;
Xhat=Xhat0;
%+sqrt(P_cov)*randn(2*(6+3*no_of_links),1);
%[X Xhat]

n=length(Xhat);
q_hat=Xhat(1:n/2);
qdot_hat=Xhat(n/2+1:n);



R_circ=20.; % radius of a circular trajectory
v_cruise=0.7; % cruise velocity m/s
w_circ=+v_cruise/R_circ; % angular velocity of cruise
center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)]; % center of the circular trajectory


samples=round(2*pi/norm(w_circ)/T) % number of samples
%samples=1
for i=1:samples
    t(i)=i*T; % time stamps
    %%%%%%%%%%%%circular trajectory generation%%%%%%%%%%%%%%%%%%
    if CIRCLE==1
        pos_des_x(i)= center(1)+R_circ*cos(w_circ*t(i)-pi/2);
        pos_des_y(i)= center(2)+R_circ*sin(w_circ*t(i)-pi/2);
        pos_des_z(i)=rc_i0(3);
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=w_circ*t(i)+etac_i0(3);%pi
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%straight line trajectory generation%%%%%%%%%%%%%%%%%%%
    if STRAIGHTLINE==1
        pos_des_x(i)=rc_i0(1)+t(i);
        pos_des_y(i)= rc_i0(2);
        pos_des_z(i)=rc_i0(3);
        roll_des(i)=etac_i0(1)*0;
        pitch_des(i)=etac_i0(2)*0;
        yaw_des(i)=etac_i0(3)*0;%pi
        
        if STOP_MIDWAY==1
            if t>10
                pos_des_x(i)=pos_des_x(i-1);
                pos_des_y(i)= rc_i0(2);
                pos_des_z(i)=rc_i0(3);
                roll_des(i)=etac_i0(1)*0;
                pitch_des(i)=etac_i0(2)*0;
                yaw_des(i)=etac_i0(3)*0;%pi
            end
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

if STRAIGHTLINE==1
    samples=1000 % samples for straight line motion
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% trajectory generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:samples
    
    if MOVING_SPARKER==1
        if CIRCLE==1
            
            if NO_OF_SPARKERS==1
                XD1=[center(1)+R_circ*cos(w_circ*t(i)-pi/2+pi/4);center(2)+R_circ*sin(w_circ*t(i)-pi/2)+pi/4;pos_des_z(i)];
                XD=XD1;
            end
            if NO_OF_SPARKERS==2
                XD1=[center(1)+R_circ*cos(w_circ*t(i)-pi/2+pi/4);center(2)+R_circ*sin(w_circ*t(i)-pi/2)+pi/4;pos_des_z(i)];
                XD2=[center(1)+R_circ*cos(w_circ*t(i)-pi/2+pi/4);center(2)+R_circ*sin(w_circ*t(i)-pi/2)+pi/4;pos_des_z(i)];
                XD=[XD1 XD2];
            end
        end
        if STRAIGHTLINE==1
            
            if NO_OF_SPARKERS==1
                XD1=[pos_des_x(i)+3;pos_des_y(i);pos_des_z(i)];
                XD=XD1;
            end
            if NO_OF_SPARKERS==2
                XD1=[pos_des_x(i)+3;pos_des_y(i);pos_des_z(i)];
                XD2=[pos_des_x(i)-20;pos_des_y(i);pos_des_z(i)];
                XD=[XD1 XD2];
            end
        end
    end
    
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
    taux=0.02*(roll_des(i)-q(4,i))-0.0*qdot(4,i); % roll control
    tauy=5*(pitch_des(i)-q(5,i))-20*qdot(5,i); % pitch control
    tauz=5*(yaw_des(i)-q(6,i))-20*qdot(6,i); % yaw control
    
    %%%%%%%%%%%% translational motion control %%%%%%%%%%%%%%%%%%%%
    fx_i=10*(pos_des_x(i)-q(1,i));%-3*qdot(1,i); % x control
    fy_i=10*(pos_des_y(i)-q(2,i));%-3*qdot(2,i); % y control
    fz_i=10*(pos_des_z(i)-q(3,i));%-3*qdot(3,i); % z control
    
    
    Tau_sol=[taux;tauy;tauz]; % control torque
    
    Tau_auv=[[fx_i;fy_i;fz_i];Tau_sol]; % force and torque control
    
    %%%%%%%%%%%%%% no control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if NO_CONTROL==1
        Tau_auv=0*Tau_auv;
    end
    
    Tau=[Tau_auv;tau_rel]; % total torque of auv and the links
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% noisy measurement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    qddot(:,i)=dir_dyn(X(1:n/2,i),X(n/2+1:n,i),Tau);
    f=[X(n/2+1:n,i)+qddot(:,i)*T;qddot(:,i)];
    X(:,i+1)=X(:,i)+f*T+sqrt(Q)*randn(2*(6+3*no_of_links),1)*T;
    
    q(:,i+1)=X(1:n/2,i+1);
    qdot(:,i+1)=X(n/2+1:n,i+1);
    i
    if NO_EKF_corr==0
        %%%%%%%%%%%%%%%%%%%%% EKF: Prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%% EKF: state prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        qddot_hat(:,i)=dir_dyn(Xhat(1:n/2,i),Xhat(n/2+1:n,i),Tau);
        f_hat=[Xhat(n/2+1:n,i)+qddot_hat(:,i)*T;qddot_hat(:,i)];
        Xhat(:,i+1)=Xhat(:,i)+f_hat*T;
        q_hat(:,i+1)=Xhat(1:n/2,i+1);
        qdot_hat(:,i+1)=Xhat(n/2+1:n,i+1);
        
        
        %%%%%%%% process Jacobian calculation %%%%%%%%%%%%%%%%%%%%%%%%%%
        delta=0.00001;
        
        for ii=1:n
            Xperturbed=Xhat(:,i);
            Xperturbed(ii)=Xperturbed(ii)+delta;
            
            qddot_pert=dir_dyn(Xperturbed(1:n/2),Xperturbed(n/2+1:n),Tau);
            f_pert=[Xperturbed(n/2+1:n)+qddot_pert*T;qddot_pert];
            
            F(:,ii)=f_pert-f_hat;
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%% Process Covariance Prediction %%%%%%%%%%%%%%%%%%%%%
        P_cov=F*P_cov*F'+Q;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%% output Jacbian calculation %%%%%%%%%%%%%%%%%%%%%%%%%%
        delta=0.00001;
        for ii=1:n
            Xperturbed=Xhat(:,i+1);
            Xperturbed(ii)=Xperturbed(ii)+delta;
            
            H(:,ii)=(op_func(Xperturbed(1:n/2),Xperturbed(n/2+1:n))-op_func(Xhat(1:n/2,i+1),Xhat(n/2+1:n,i+1)))/delta;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% EKF: Update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%% output noise covariance %%%%%%%%%%%%%%%%%%%%%%%%%%
        Rv=op_noise_cov(X(1:n/2,i));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%% Kalman Gain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        K=P_cov*H'*inv(H*P_cov*H'+Rv);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%% State uapdate %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        delta_y=op_noise(X(1:n/2,i+1),X(n/2+1:n,i+1))-op_func(Xhat(1:n/2,i+1),Xhat(n/2+1:n,i+1));
        
        Xhat(:,i+1)=Xhat(:,i+1)+K*delta_y;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%% Process covariance update %%%%%%%%%%%%%%%%%%%%%%%%%
        P_cov=(eye(2*(6+3*no_of_links))-K*H)*P_cov;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        q_hat(:,i+1)=Xhat(1:n/2,i+1);
        qdot_hat(:,i+1)=Xhat(n/2+1:n,i+1);
        %i
        eig_cov(:,i+1)=eig(P_cov); % eigen values of process covariance matrix
    end
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            q(7+3*j,i)=0;
            qdot(7+3*j,i)=0;
            qddot(7+3*j,i)=0;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%% Animation of the simulation    %%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ANIMATION==1
        
        cla
        hold on
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
        drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
        
        [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,i),qdot_hat(:,i),qddot_hat(:,i));
        
        
        DrawPlot(P_draw,R_draw,0)
        DrawPlot(P_draw_hat,R_draw_hat,1)
        
        plot3(XD(1,:),XD(2,:),XD(3,:),'*')
        plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
        
        axis([-50 50 -50 50 -50 50])
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%% Calculation of the norm of the error of the positions %%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%
    if NORM_POSITION==1
        
        [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,i),qdot_hat(:,i),qddot_hat(:,i));
        errn_link(1,:)=norm(P_draw(:,1)-P_draw_hat(:,1));
        errn_link(2,:)=norm(P_draw(:,2)-P_draw_hat(:,2));
        for k=1:no_of_links
            errn_link(2+k,:)=norm(P_draw(:,2+k)-P_draw_hat(:,2+k));
        end
        pos_hyd_errnrm(:,i)=errn_link;
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%%%%%%%%%%%%%% plot of the system at different time %%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%
if FEW_SAMPLES==1
    cla
    hold on
    [row,col]=size(q);
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,1),qdot(:,1),qddot(:,1));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q_hat(:,1),qdot_hat(:,1),qddot_hat(:,1));
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/4)),qdot(:,round(col/4)),qddot(:,round(col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q_hat(:,round(col/4)),qdot_hat(:,round(col/4)),qddot_hat(:,round(col/4)));
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(col/2)),qdot(:,round(col/2)),qddot(:,round(col/2)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q_hat(:,round(col/2)),qdot_hat(:,round(col/2)),qddot_hat(:,round(col/2)));
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,round(3*col/4)),qdot(:,round(3*col/4)),qddot(:,round(3*col/4)));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q_hat(:,round(3*col/4)),qdot_hat(:,round(3*col/4)),qddot_hat(:,round(3*col/4)));
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    plot3(XD(1,:),XD(2,:),XD(3,:),'*')
    plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
    
    hold off
    
    %axis square
    axis equal
    if STRAIGHTLINE==1
        %axis([-15 samples*T -2 2 -0 4])
        axis([-15 samples*T -30 30 -10 10])
    end
    if CIRCLE==1
        axis([-20 20 -10 50 -10 10])
    end
    
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    %title(sprintf('Time: %0.2f sec', t(i)));
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% plot of the eigen values %%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if EIG_PLOT==1
    [row,col]=size(eig_cov);
    figure
    plot(t(1:samples),eig_cov(1:row,1:col-1),'LineWidth',2);
    xlabel('t [s]','FontSize',12);
    ylabel('Eigen Values','FontSize',12);
    set(gca,'fontsize',12)
    
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'eigenval.jpg')
        saveas(gcf,'eigenval','epsc')
        cd ..
    end
    %     figure
    %     plot(t,eig_cov(row/2+1:row,1:col-1));
    %     xlabel('t [s]');
    %     ylabel('Eigen Values');
    %     if SAVE_FIG==1
    %         cd img
    %         saveas(gcf,'err_auv_pos_vel.jpg')
    %         cd ..
    %     end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot of error %%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ERR_PLOT==1
    
    %% error plot
    
    Xe=abs(X-Xhat);
    [row,col]=size(Xe);
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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

%%%%%%%%%%%% plot of error norm %%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if NORM_POSITION==1
    
    figure
    plot(t(1:samples),pos_hyd_errnrm)
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('error norm [m]','FontSize',12);
    str=[];
    str=[str,cellstr(char(strcat('Point C ')))];
    str=[str,cellstr(char(strcat('Point P ')))];
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

if SAVE_FILE==1
    if MOVING_SPARKER==1
        if CIRCLE==1
            if NO_OF_SPARKERS==1
                save('auvstreamerDataCircMovOneSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
            if NO_OF_SPARKERS==2
                save('auvstreamerDataCircMovTwoSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
        end
        if STRAIGHTLINE==1
            if NO_OF_SPARKERS==1
                save('auvstreamerDataStrlnMovOneSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
            if NO_OF_SPARKERS==2
                save('auvstreamerDataStrlnMovTwoSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
        end
    else
        if CIRCLE==1
            if NO_OF_SPARKERS==1
                save('auvstreamerDataCircOneSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
            if NO_OF_SPARKERS==2
                save('auvstreamerDataCircTwoSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
            end
        end
        if STRAIGHTLINE==1
            
            if STOP_MIDWAY==1
                if NO_OF_SPARKERS==1
                    save('auvstreamerDataStrlnOneSPSTOP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
                end
                if NO_OF_SPARKERS==2
                    save('auvstreamerDataStrlnTwoSPSTOP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
                end
            else
                if NO_OF_SPARKERS==1
                    save('auvstreamerDataStrlnOneSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
                end
                if NO_OF_SPARKERS==2
                    save('auvstreamerDataStrlnTwoSP.mat','q','qdot','qddot','q_hat','qdot_hat','qddot_hat','t','eig_cov','X','Xhat','Xe','pos_hyd_errnrm')
                end
            end
        end
    end
end

if var_bool==1
    plotofdata(q,qdot,qddot,samples,t);
end
