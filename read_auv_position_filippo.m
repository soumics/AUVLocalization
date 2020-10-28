%%%%%%%%%%%%%% experimental data extraction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
close all
clear all

global t_tmp;
global range;
global SIMULATION;
SIMULATION=0;
RANGE_CORRECTION=8;
ANIMATION=0; % 1 to watch animation else 0
var_bool=0; % 1 to see position, velocity, acceleration, force w.r.t all frames else 0
ERR_PLOT=1; % 1 to see error plot else 0
EIG_PLOT=1; % 1 to see eigen value plot else 0
ACT_DATA_PLOT=0; % 1 to see actual data plot else 0
EST_DATA_PLOT=0; % 1 to see estimated data plot else 0
NORM_POSITION=1; % 1 to see the norm position else 0
POSITION_PLOT=1; % 1 to see the positions else 0

FEW_SAMPLES=1; % 1 to plot a few samples else 0

CIRCLE=0; % 1 to extract results for circular trajectory else 0
STRAIGHTLINE=1; % 1 to extract results for straight line trajectory else 0

MOVING_SPARKER=0; % 1 to extract results for moving saprkers else 0

STOP_MIDWAY=0; % 1 to extract results when the auv stops in the midway else 0

SAVE_FIG=1; % 1 to save figures else 0
SAVE_FILE=1; % 1 to save to file else 0
NO_OF_SPARKERS=1; % no of sparkers 1 0r 2

NO_ROLL=1; % 0 for no roll else 1
NO_YAW=0; % 0 for no yaw else 1
NO_CONTROL=0; % 0 for no control else 1
NO_EKF_corr=0; % 0 for no EKF correction else 1, ie, only prediction

Param % invoking all parameters from Param.m



if SIMULATION==0
    %%%%% READ DATA OF MEDUSA RED GPS
    S = load('mr_data_5.mat'); %%Medusa Red data GPS position. starting time 17:2:26.94 terminate 17:26:58.63
    %t=S.data(:,1);
%https://www.epochconverter.com/timezones?q=1480093200&tz=Europe%2FLisbon
% 1480093200 correspondes to November 25, 2016 17:00:00 (pm)

    initial_time_gps=S.data(1,1);%-1480093200 %+17*3600; from 17:00:00
    time_var_temp=S.data(1,1)-1480093200
    disp(['initial time: 17:' num2str(fix(time_var_temp/60)) ':' num2str(time_var_temp-fix(time_var_temp/60)*60)])
    S.data(:,1)=S.data(:,1)-S.data(1,1)
    off_set_x=512740;
    off_set_y=4198300;
    S.data(:,5)= S.data(:,5)-off_set_y
    S.data(:,6)= S.data(:,6)-off_set_x
    %%%%% PARSING OF THE FILE TO CONNECT TIMING AND FFID 
    fileID = fopen('mr.500ms.merge_mobile.log','r');
    
    %%  Fixed source coordinates: Easting: 512716.72, Northing: 4198376.63
    
    Block = 1;
    plot_MS=1;
    i=0
    while (~feof(fileID))                               % For each block:
        i=i+1;        
        timing = textscan(fileID,'%s',1,'delimiter',' ');  % Read 2 header lines
        time_string=timing{1};
        HeaderLines{Block,1} = timing{1};
        hour(i)=str2num(time_string{1}(1:2));
        minute(i)=str2num(time_string{1}(4:5));
        second(i)=str2num(time_string{1}(7:11));
        InputText = textscan(fileID,'SOU_FFID %f') ;
        HeaderLines{Block,1} = InputText{1};
        sou_FFID(i)=cell2mat(InputText);
        InputText = textscan(fileID,'MR_FFID %f') ;
        HeaderLines{Block,1} = InputText{1};
        mr_FFID(i)=cell2mat(InputText);        
        out_mat(i,1)=(hour(i)-17)*3600+minute(i)*60+second(i)-(initial_time_gps-1480093200);%(hour(1)*3600+minute(1)*60+second(1));
        
        out_mat(i,2)=sou_FFID(i);
        out_mat(i,3)=mr_FFID(i);
        out_mat(i,4)=hour(i)*3600+minute(i)*60+second(i);
        
        Block = Block+1;                                 % Increment block index
    end
    disp('initial_time_gps-out_mat(1,4)')
    initial_time_gps-out_mat(1,4)
    
    MR_ranges=load('MR_ranges.dat');
     for i=1:8
        range{i}.mob=[];
        range{i}.mob_m=[];
        range{i}.stat=[];
        range{i}.stat_m=[];
        range{i}.time=[];
    end
    sound_speed=1513;
    mov_src_x=[];
    mov_src_y=[];
    mov_src_time=[];    
    
    for i=1:size(MR_ranges,1)
        MR_ranges(i,1) %FFIDn
        MR_ranges(i,2) %chan
        MR_ranges(i,7) %range from mob
        MR_ranges(i,7); %range froms tatic
        if (i>1 & MR_ranges(i,3)~=MR_ranges(i-1,3) & MR_ranges(i,4)~=MR_ranges(i-1,4))
            mov_src_x=[mov_src_x;MR_ranges(i,3)-off_set_x];
            mov_src_y=[mov_src_y;MR_ranges(i,4)-off_set_y];
            index=find(out_mat(:,3)==MR_ranges(i,1));
            time_stamp=1/10*fix(10*out_mat(index,1));
            mov_src_time=[mov_src_time;time_stamp];     
        end       
        range{MR_ranges(i,2)}.mob=[range{MR_ranges(i,2)}.mob;MR_ranges(i,7)];
        range{MR_ranges(i,2)}.mob_m=[range{MR_ranges(i,2)}.mob_m;MR_ranges(i,7)/1000*sound_speed];
        
        range{MR_ranges(i,2)}.stat=[range{MR_ranges(i,2)}.stat;MR_ranges(i,8)];
        range{MR_ranges(i,2)}.stat_m=[range{MR_ranges(i,2)}.stat_m;MR_ranges(i,8)/1000*sound_speed+RANGE_CORRECTION];
        index=find(out_mat(:,3)==MR_ranges(i,1));
        time_stamp=1/10*fix(10*out_mat(index,1));
        range{MR_ranges(i,2)}.time=[range{MR_ranges(i,2)}.time;time_stamp];
        
    end
end
% if NO_OF_SPARKERS==1
%     if CIRCLE==1
%         XD1=[0;20;0];
%         XD=XD1;
%     end
%     if STRAIGHTLINE==1
%         XD1=[20;20;0];
%         XD=XD1;
%     end
% end
%
% if NO_OF_SPARKERS==2
%     XD1=[20;20;0];
%     XD2=[20;-20;0];
%     XD=[XD1 XD2];
% end
%%% SPARKER POSITION
XD=[512716.72-off_set_x;4198376.63-off_set_y;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% initial configuration of the AUV and streamer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rc_i0=[S.data(1,6);S.data(1,5);S.data(1,7)]; % initial position of point C
etac_i0=[0*pi/2;0*pi/4;0*pi+0*pi/4]; % Euler angles of C w.r.t I [roll, pitch, yaw]

eta_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler angles between two consecutive frmes
%eta_rel0(1)=0
%eta_rel0(2)=pi/3
%eta_rel0(3)=-pi/100
%eta_rel0(4)=-pi/4
vc_i0=[0;0;0]; % initial velocity of point C
etadc_i0=[0;0;0]; % Eular rate of C w.r.t I
etad_rel0=kron(ones(no_of_links,1),[0;0;0]); % inital relative Euler rates between two consecutive frmes
%%%% state initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q0=[rc_i0;etac_i0;eta_rel0];
qd0=[vc_i0;etadc_i0;etad_rel0];
qdd0=zeros(6+3*no_of_links,1);
q=q0; qdot=qd0; qddot=qdd0;
X0=[q;qdot]; % initial state vector
X=X0;

fx=0.0; fy=0; fz=0; taux=0.0; tauy=0; tauz=0; % forces and torques on auv

tau_rel_link=[0;0;0];
tau_rel=kron(ones(no_of_links,1),tau_rel_link); % torques on links

Tau_auv=[[fx;fy;fz];[taux;tauy;tauz]]; % forces and torques on AUV
Tau=[Tau_auv;tau_rel]; % forces and torques on auv and the links


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% EKF  covariance initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%P_cov=0.1*eye(2*(6+3*no_of_links)); % Process covariance initialization
%P_cov=blkdiag(0.1*eye(3),.0012*eye(3),.0012*eye(3*no_of_links),0.1*eye(3),.0012*eye(3),.0012*eye(3*no_of_links));
P_cov=blkdiag(.5*eye(3),.0024*eye(3),.0024*eye(3*no_of_links),0.01*eye(3),0.0001*eye(3),0.0001*eye(3*no_of_links));
%P_cov=eye(2*(6+3*no_of_links));

%  for p=1:no_of_links
%      P_cov(4+3*p,4+3*p)=0.00001;
%      P_cov(10+3*no_of_links+3*p,10+3*no_of_links+3*p)=0.00001;
% end
%P_cov

%P_cov=0*P_cov;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
eig_cov=eig(P_cov); % eigen value of process covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% process error covariance  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Q=blkdiag(0.001*eye(3),.01*(2/180)*pi*eye(3),.001*(2/180)*pi*eye(3*no_of_links),0.001*eye(3),.001*(2/180)*pi*eye(3),.001*(2/180)*pi*eye(3*no_of_links));
Q=blkdiag(0.5*eye(3),.0012*eye(3),.0012*eye(3*no_of_links),0.01*eye(3),.0001*eye(3),.0001*eye(3*no_of_links));
%Q=blkdiag(0.01*eye(3),.01*eye(3),.00001*eye(3*no_of_links),0.01*eye(3),.01*eye(3),.00001*eye(3*no_of_links));

% for p=1:no_of_links
%     Q(4+3*p,4+3*p)=0.00001;
%     Q(6+3*no_of_links+4+3*p,10+3*no_of_links+3*p)=0.00001;
% end


%Q=0*Q;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



samples=2500%00;%000%7360
T=.1; % sampling interval

%%%%%%%%%% EKF STATE INITIALIZATION
qddot_hat=qddot;
Xhat0=X0;
Xhat=Xhat0+sqrt(P_cov)*randn(2*(6+3*no_of_links),1);

Xhat(8)=Xhat(8)+pi/10
%[X Xhat]
n=length(Xhat);
q_hat=Xhat(1:n/2);
qdot_hat=Xhat(n/2+1:n);



% R_circ=20.; % radius of a circular trajectory
% v_cruise=0.7; % cruise velocity m/s
% w_circ=+v_cruise/R_circ; % angular velocity of cruise
% center=[rc_i0(1) rc_i0(2)+R_circ rc_i0(3)]; % center of the circular trajectory
%
%
% samples=round(2*pi/norm(w_circ)/T) % number of samples
%samples=length(S.data(:,1))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% trajectory generation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%pos_des_x= S.data(:,6);

%%%%% GENERATE THE DESIRED PATH FROM REAL GPS MEASURMENT OF THE 

for i=1:samples
    i
    t(i)=i*T;
    [val index]=min(abs(S.data(:,1)-t(i)))
    t(i)
    S.data(index-1,1)
    S.data(index,1)
    S.data(index-1,6)
    S.data(index,6)    
    if i==1
        if t(i)>S.data(index,1)
            pos_des_x(i)=S.data(index,6)
            pos_des_y(i)=S.data(index,5)
        else
            pos_des_x(i)=S.data(index+1,6)
            pos_des_y(i)=S.data(index+1,5)
        end
    else
        if t(i)>S.data(index,1)
            pos_des_x(i)=S.data(index,6)+(S.data(index+1,6)-S.data(index,6))*(t(i)-S.data(index,1))/(S.data(index+1,1)-S.data(index,1));
            pos_des_y(i)=S.data(index,5)+(S.data(index+1,5)-S.data(index,5))*(t(i)-S.data(index,1))/(S.data(index+1,1)-S.data(index,1));
        end
        if t(i)<S.data(index,1)
            pos_des_x(i)=S.data(index-1,6)+(S.data(index,6)-S.data(index-1,6))*(t(i)-S.data(index-1,1))/(S.data(index,1)-S.data(index-1,1));
            pos_des_y(i)=S.data(index-1,5)+(S.data(index,5)-S.data(index-1,5))*(t(i)-S.data(index-1,1))/(S.data(index,1)-S.data(index-1,1));
        end
    end
    pos_des_z(i)=0
end

roll_des=pos_des_x*0;
pitch_des=pos_des_x*0;
yaw_des=[];
for i=1:length(pos_des_x)
    if i==length(pos_des_x)
        yaw_des=[yaw_des;yaw_des(i-1)];
    else
        yaw_ang_temp=atan2(pos_des_y(i+1)-pos_des_y(i),pos_des_x(i+1)-pos_des_x(i));
        yaw_des=[yaw_des;yaw_ang_temp];
    end
end
yaw_des=[yaw_des;yaw_ang_temp];


for i=1:samples
    
    t(i)=i*T; % time stamps
    t_tmp=t(i);
    
      
    %%%%%%%%%%%%% angular motion control %%%%%%%%%%%%%%%%%%%%%%%%
    taux=0.02*(roll_des(i)-q(4,i))-0.0*qdot(4,i); % roll control
    tauy=5*(pitch_des(i)-q(5,i))-20*qdot(5,i); % pitch control
    tauz=5*(yaw_des(i)-q(6,i))-20*qdot(6,i); % yaw control
    
    %%%%%%%%%%%% translational motion control %%%%%%%%%%%%%%%%%%%%
    fx_i=10*(pos_des_x(i)-q(1,i));%-3*qdot(1,i); % x control
    fy_i=10*(pos_des_y(i)-q(2,i));%-3*qdot(2,i); % y control
    fz_i=10*(pos_des_z(i)-q(3,i));%-3*qdot(3,i); % z control
    
    % [pos_des_x(i+1),pos_des_y(i+1),pos_des_z(i+1)]-[pos_des_x(1),pos_des_y(1),pos_des_z(1)]
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
    
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            X(7+3*j,i)=0;
            X(n/2+7+3*j,i)=0;
            % q(7+3*j,i)=0;
            % qdot(7+3*j,i)=0;
        end
    end
    
    qddot(:,i)=dir_dyn(X(1:n/2,i),X(n/2+1:n,i),Tau);
    f=[X(n/2+1:n,i)+qddot(:,i)*T;qddot(:,i)];    
    
    X(:,i+1)=X(:,i)+f*T+sqrt(Q)*randn(2*(6+3*no_of_links),1)*T;
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            X(7+3*j,i+1)=0;
            X(n/2+7+3*j,i+1)=0;
            % q(7+3*j,i)=0;
            % qdot(7+3*j,i)=0;
        end
    end
    
    q(:,i+1)=X(1:n/2,i+1);
    qdot(:,i+1)=X(n/2+1:n,i+1);
    %     if NO_ROLL==1
    %         for j=no_of_links-1:-1:0
    %             q(7+3*j,i)=0;
    %             qdot(7+3*j,i)=0;
    %             qddot(7+3*j,i)=0;
    %         end
    %     end
    i
    %%%%%%%%%%%%%%%%%%%%% EKF: Prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% EKF: state prediction %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            Xhat(7+3*j,i)=0;
            Xhat(n/2+7+3*j,i)=0;
            % q(7+3*j,i)=0;
            % qdot(7+3*j,i)=0;
        end
    end
    
    qddot_hat(:,i)=dir_dyn(Xhat(1:n/2,i),Xhat(n/2+1:n,i),Tau);
    f_hat=[Xhat(n/2+1:n,i)+qddot_hat(:,i)*T;qddot_hat(:,i)];
    Xhat(:,i+1)=Xhat(:,i)+f_hat*T;
    
    if NO_ROLL==1
        for j=no_of_links-1:-1:0
            Xhat(7+3*j,i+1)=0;
            Xhat(n/2+7+3*j,i+1)=0;
            % q(7+3*j,i)=0;
            % qdot(7+3*j,i)=0;
        end
    end
    %     if NO_ROLL==1
    %         for j=no_of_links-1:-1:0
    %             Xhat(7+3*j,i+1)=0;
    %             Xhat(n/2+7+3*j:n,i+1)=0;
    %            % q(7+3*j,i)=0;
    %            % qdot(7+3*j,i)=0;
    %         end
    %     end
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
    if NO_EKF_corr==0
        
        %%%%%%%%% output Jacbian calculation %%%%%%%%%%%%%%%%%%%%%%%%%%
        H=[];
        delta=0.00001;
        for ii=1:n
            Xperturbed=Xhat(:,i+1);
            Xperturbed(ii)=Xperturbed(ii)+delta;
            
            H(:,ii)=(op_func(Xperturbed(1:n/2),Xperturbed(n/2+1:n),i)-op_func(Xhat(1:n/2,i+1),Xhat(n/2+1:n,i+1),i))/delta;
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% EKF: Update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%% output noise covariance %%%%%%%%%%%%%%%%%%%%%%%%%%
        Rv=op_noise_cov(X(1:n/2,i),i);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%% Kalman Gain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        K=P_cov*H'*inv(H*P_cov*H'+Rv);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%% State uapdate %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        delta_y=op_noise(X(1:n/2,i+1),X(n/2+1:n,i+1),i)-op_func(Xhat(1:n/2,i+1),Xhat(n/2+1:n,i+1),i);
        
        Xhat(:,i+1)=Xhat(:,i+1)+K*delta_y;
        
        if NO_ROLL==1
            for j=no_of_links-1:-1:0
                Xhat(7+3*j,i+1)=0;
                Xhat(n/2+7+3*j,i+1)=0;
                % q(7+3*j,i)=0;
                % qdot(7+3*j,i)=0;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%% Process covariance update %%%%%%%%%%%%%%%%%%%%%%%%%
        P_cov=(eye(2*(6+3*no_of_links))-K*H)*P_cov;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        q_hat(:,i+1)=Xhat(1:n/2,i+1);
        qdot_hat(:,i+1)=Xhat(n/2+1:n,i+1);
        %i
        eig_cov(:,i+1)=eig(P_cov); % eigen values of process covariance matrix
    end
    
    %       X(:,i+1)';
    %     Xhat(:,i+1)'
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
        plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',1)
        
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
        
        i
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
    
    
    [P_draw,R_draw,V_draw,W_draw,Vd_draw,Wd_draw]=acc_angacc(q(:,col-1),qdot(:,col-1),qddot(:,col-1));
    eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw,R_draw,0)
    [P_draw_hat,R_draw_hat,V_draw_hat,W_draw_hat,Vd_draw_hat,Wd_draw_hat]=acc_angacc(q_hat(:,col-1),qdot_hat(:,col-1),qddot_hat(:,col-1));
    %eularAng=rad2deg(Eular_ang(R_draw(:,:,1))');
    %drawEllipsoid([P_draw(:,1)' 1 .25 .25 eularAng(3) eularAng(2) eularAng(1)]);
    DrawPlot(P_draw_hat,R_draw_hat,1)
    
    
    plot3(XD(1,:),XD(2,:),XD(3,:),'*')
    plot3(pos_des_x,pos_des_y,pos_des_z,'r','LineWidth',2)
    
    hold off
    
    axis square
    axis equal
    
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    %title(sprintf('Time: %0.2f sec', t(i)));
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'errnorm.jpg')
        saveas(gcf,'complete_path','epsc')
        cd ..
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% plot of the eigen values %%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if EIG_PLOT==1
    [row,col]=size(eig_cov);
    figure
    plot(t(1:samples),eig_cov(1:row,1:col-1),'LineWidth',1);
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
Xe=abs(X-Xhat);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot of error %%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ERR_PLOT==1
    
    %% error plot
    
    
    [row,col]=size(Xe);
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xe(1:3,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{r}}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xe(row/2+1:row/2+1+2,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\mathbf{v}}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    SAVE_FIG
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'err_auv_pos_vel.jpg')
        saveas(gcf,'err_auv_pos_vel','epsc')
        cd ..
    end
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xe(4:6,1:col-1),'LineWidth',1)
    legend(thisax,{'$\tilde{\phi}_c^i$','$\tilde{\theta}_c^i$','$\tilde{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\tilde{\eta}_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xe(row/2+1+3:row/2+1+5,1:col-1),'LineWidth',1)
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
            
            plot(t(1:samples),Xe(7+3*(i-1):6+3*i,1:col-1),'LineWidth',1)
            legend(thisax,{['$\tilde{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\tilde{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\tilde{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            
            thisax = subplot(2,1,2);
            plot(t(1:samples),Xe((row/2+6+1+3*(i-1)):(row/2+6+3*i),1:col-1),'LineWidth',1)
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
    plot(t(1:samples),X(1:3,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\mathbf{r}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),X(row/2+1:row/2+1+2,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\mathbf{v}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),X(4:6,1:col-1),'LineWidth',1)
    legend(thisax,{'$\phi_c^i$','$\theta_c^i$','$\psi_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\eta_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),X(row/2+1+3:row/2+1+5,1:col-1),'LineWidth',1)
    legend(thisax,{'$\dot{\phi}_c^i$','$\dot{\theta}_c^i$','$\dot{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\dot{\eta}_c^i \ [rad/s]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    if no_of_links>0
        for i=1:no_of_links
            figure
            thisax = subplot(2,1,1);
            
            plot(t(1:samples),X(7+3*(i-1):6+3*i,1:col-1),'LineWidth',1)
            legend(thisax,{['$\phi_' num2str(i) '^' num2str(i-1) '$'],['$\theta_' num2str(i) '^' num2str(i-1) '$'],['$\psi_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\tilde{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            thisax = subplot(2,1,2);
            plot(t(1:samples),X((row/2+6+1+3*(i-1)):(row/2+6+3*i),1:col-1),'LineWidth',1)
            legend(thisax,{['$\dot{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\dot{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad/s]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%% plot of estimated data %%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if EST_DATA_PLOT==1
    %% plot of actual data
    
    
    [row,col]=size(Xhat);
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xhat(1:3,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\hat{\mathbf{r}}_C^i \ [m]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xhat(row/2+1:row/2+1+2,1:col-1),'LineWidth',1)
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\hat{\mathbf{v}}_C^i \ [m/s]$','Interpreter','LaTex','FontSize',16);
    %ylabel('AUV Centroid: velocity error [m]','FontSize',12);
    set(gca,'fontsize',12)
    
    
    figure
    thisax = subplot(2,1,1);
    plot(t(1:samples),Xhat(4:6,1:col-1),'LineWidth',1)
    legend(thisax,{'$\hat{\phi}_c^i$','$\hat{\theta}_c^i$','$\hat{\psi}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\hat{\eta}_c^i \ [rad]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    thisax = subplot(2,1,2);
    plot(t(1:samples),Xhat(row/2+1+3:row/2+1+5,1:col-1),'LineWidth',1)
    legend(thisax,{'$\dot{\hat{\phi}}_c^i$','$\dot{\hat{\theta}}_c^i$','$\dot{\hat{\psi}}_c^i$'}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('$\dot{\hat{\eta}}_c^i \ [rad/s]$','Interpreter','LaTex','FontSize',16);
    set(gca,'fontsize',12)
    
    if no_of_links>0
        for i=1:no_of_links
            figure
            thisax = subplot(2,1,1);
            
            plot(t(1:samples),Xhat(7+3*(i-1):6+3*i,1:col-1),'LineWidth',1)
            legend(thisax,{['$\hat{\phi}_' num2str(i) '^' num2str(i-1) '$'],['$\hat{\theta}_' num2str(i) '^' num2str(i-1) '$'],['$\hat{\psi}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\hat{\eta}_' num2str(i) '^' num2str(i-1) ' \ [rad]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
            
            thisax = subplot(2,1,2);
            plot(t(1:samples),Xhat((row/2+6+1+3*(i-1)):(row/2+6+3*i),1:col-1),'LineWidth',1)
            legend(thisax,{['$\dot{\hat{\phi}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\hat{\theta}}_' num2str(i) '^' num2str(i-1) '$'],['$\dot{\hat{\psi}}_' num2str(i) '^' num2str(i-1) '$']}, 'Interpreter', 'LaTex','FontSize',12, 'location', 'northeast')
            xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
            ylabel(['$\dot{\hat{\eta}}_' num2str(i) '^' num2str(i-1) ' \ [rad/s]$'], 'Interpreter','LaTex','FontSize',16);
            set(gca,'fontsize',12)
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%% plot of positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if POSITION_PLOT==1
    
    for i=1:samples-1
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
    
    %     plot3(P_draw_X(:,1),P_draw_Y(:,1),P_draw_Z(:,1))
    %
    %     plot3(P_draw_hat_X(:,1),P_draw_hat_Y(:,1),P_draw_hat_Z(:,1),'k--')
    
    for plt=1:no_of_links+2
        plot3(P_draw_X(:,plt),P_draw_Y(:,plt),P_draw_Z(:,plt))
        
        plot3(P_draw_hat_X(:,plt),P_draw_hat_Y(:,plt),P_draw_hat_Z(:,plt),'k--')
    end
    
    set(gca, 'XDir', 'reverse')
    set(gca, 'ZDir', 'reverse')
    grid on
    xlabel('$X [m]$','Interpreter','LaTex','FontSize',12);
    ylabel('$Y [m]$','Interpreter','LaTex','FontSize',12);
    zlabel('$Z [m]$','Interpreter','LaTex','FontSize',12);
    if SAVE_FIG==1
        cd img
        % saveas(gcf,'errnorm.jpg')
        saveas(gcf,'path','epsc')
        cd ..
    end
end


%%%%%%%%%%%% plot of error norm %%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if NORM_POSITION==1
    
    figure
    %subplot 211
    plot(t(1:samples),pos_hyd_errnrm)
    xlabel('$t [s]$','Interpreter','LaTex','FontSize',16);
    ylabel('estim error norm [m]','Interpreter','LaTex','FontSize',12);
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


figure
for i=1:no_of_links
subplot(4,2,i)
plot(range{i}.stat_m)
ylim([0 70])
end