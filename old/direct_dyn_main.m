clc;
close all;
clear all;
% hold on;
ANIMATION=1;
Param


fxc_c=0; fyc_c=0; fzc_c=0; tauxic_c=0; tauyic_c=1; tauzic_c=0; % force and torque of auv

tau_rel_single=[0;0;0];
tau_rel=kron(ones(no_of_links,1),tau_rel_single);

% tau_rel(3)=pi/50;
% tau_rel(6)=-pi/50;

u=[fxc_c;fyc_c;fzc_c;tauxic_c;tauyic_c;tauzic_c;tau_rel];


rc_i0=[1;1;1]; % initial position of point C

etac_i0=[0;0;pi/5]; % Eular angles of C w.r.t I

eta_rel=kron(ones(no_of_links,1),[0;0;0]); 

vc_i0=[0;0;0]; % initial velocity of point C

etadc_i0=[0;0;0]; % Eular rate of C w.r.t I

etad_rel=kron(ones(no_of_links,1),[0;0;0]); 


q0=[rc_i0;etac_i0;eta_rel];
qd0=[vc_i0;etadc_i0;etad_rel];
qdd0=zeros(6+3*no_of_links,1);
q=q0; qdot=qd0; qddot=qdd0;

T=.1;
samples=350;
for i=1:samples
    t(i)=i*T;
    if t(i)>10
      u=0*u;  
    end
     %if t<=100
%         ud(5)=pi/10;
%     end
%     if t>100
%         ud(5)=0;
%         ud(6)=pi/10;
%     end
    
    qddot(:,i+1)=direct_kin(q(:,i),qdot(:,1),u);
    qdot(:,i+1)=qddot(:,i+1)*T+qdot(:,i);
    q(:,i+1)=qdot(:,i+1)*T+q(:,i);
    
    [r,R]=pos_rot_pnts(q(1:6+3*no_of_links,i));
    if ANIMATION==1
        cla
        hold on
        DrawPlot(r,R)
        hold off
        
        
        
        axis([-10 10 -10 10 -10 10])
        axis square
        set(gca, 'XDir', 'reverse')
        set(gca, 'ZDir', 'reverse')
        title(sprintf('Time: %0.2f sec', t(i)));
        grid on
        % view([0,90])
        %hold on;
        pause(.01)
    end
    
end


for i=1:samples
    [a b]=pos_rot_pnts(q(:,i));
    
    P_link(1).pos(:,i)=a(:,1);
    P_link(2).pos(:,i)=a(:,2);
    for j=1:no_of_links
        P_link(2+j).pos(:,i)=a(:,2+j);
    end 
end

figure
subplot(2+no_of_links,1,1)
hold on
plot(t,P_link(1).pos(1,:),'r')
plot(t,P_link(1).pos(2,:),'g')
plot(t,P_link(1).pos(3,:),'b')
hold off

subplot(2+no_of_links,1,2)
hold on
plot(t,P_link(2).pos(1,:),'r')
plot(t,P_link(2).pos(2,:),'g')
plot(t,P_link(2).pos(3,:),'b')
hold off

for i=1:no_of_links
    subplot(2+no_of_links,1,2+i)
    hold on
    
    plot(t,P_link(2+i).pos(1,:),'r')
    plot(t,P_link(2+i).pos(2,:),'g')
    plot(t,P_link(2+i).pos(3,:),'b')
    hold off
end



for i=1:samples
    [a1 b1]=vel_angvel(q(:,i),qdot(:,i));
%     vel_V(:,i)=a1(:,1);
%     angvel_W(:,i)=b1(:,1);
    P_link(1).vel(:,i)=a1(:,1);
    P_link(1).angvel(:,i)=b1(:,1);
    P_link(2).vel(:,i)=a1(:,2);
    P_link(2).angvel(:,i)=b1(:,2);
    
    for j=1:no_of_links
        P_link(2+j).vel(:,i)=a1(:,2+j);
        P_link(2+j).angvel(:,i)=b1(:,2+j);
        
    end 
end

figure
subplot(2+no_of_links,2,1);
    hold on
    
    plot(t,P_link(1).vel(1,:),'r')
    plot(t,P_link(1).vel(2,:),'g')
    plot(t,P_link(1).vel(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,2);
    hold on
    plot(t,P_link(1).angvel(1,:),'r')
    plot(t,P_link(1).angvel(2,:),'g')
    plot(t,P_link(1).angvel(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,3);
    hold on
    
    plot(t,P_link(2).vel(1,:),'r')
    plot(t,P_link(2).vel(2,:),'g')
    plot(t,P_link(2).vel(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,4);
    hold on
    plot(t,P_link(2).angvel(1,:),'r')
    plot(t,P_link(2).angvel(2,:),'g')
    plot(t,P_link(2).angvel(3,:),'b')
    hold off

k=4;
for i=1:no_of_links
    k=k+1;
    subplot(2+no_of_links,2,k);
    hold on
    
    plot(t,P_link(2+i).vel(1,:),'r')
    plot(t,P_link(2+i).vel(2,:),'g')
    plot(t,P_link(2+i).vel(3,:),'b')
    hold off
    
    k=k+1;
    subplot(2+no_of_links,2,k);
    hold on
    plot(t,P_link(2+i).angvel(1,:),'r')
    plot(t,P_link(2+i).angvel(2,:),'g')
    plot(t,P_link(2+i).angvel(3,:),'b')
    hold off
end


for i=1:samples
    [a2 b2]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
%     acc_Vd(:,i)=a2(:,1);
%     angacc_Wd(:,i)=b2(:,1);
    
    P_link(1).acc(:,i)=a2(:,1);
    P_link(1).angacc(:,i)=b2(:,1);
    P_link(2).acc(:,i)=a2(:,2);
    P_link(2).angacc(:,i)=b2(:,2);
    
    for j=1:no_of_links
        P_link(2+j).acc(:,i)=a2(:,2+j);
        P_link(2+j).angacc(:,i)=b2(:,2+j);
        
    end 
end

figure
subplot(2+no_of_links,2,1);
    hold on
    
    plot(t,P_link(1).acc(1,:),'r')
    plot(t,P_link(1).acc(2,:),'g')
    plot(t,P_link(1).acc(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,2);
    hold on
    plot(t,P_link(1).angacc(1,:),'r')
    plot(t,P_link(1).angacc(2,:),'g')
    plot(t,P_link(1).angacc(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,3);
    hold on
    
    plot(t,P_link(2).acc(1,:),'r')
    plot(t,P_link(2).acc(2,:),'g')
    plot(t,P_link(2).acc(3,:),'b')
    hold off
    
    subplot(2+no_of_links,2,4);
    hold on
    plot(t,P_link(2).angacc(1,:),'r')
    plot(t,P_link(2).angacc(2,:),'g')
    plot(t,P_link(2).angacc(3,:),'b')
    hold off
    
k=4;
for i=1:no_of_links
    k=k+1;
    subplot(2+no_of_links,2,k);
    hold on
    
    plot(t,P_link(2+i).acc(1,:),'r')
    plot(t,P_link(2+i).acc(2,:),'g')
    plot(t,P_link(2+i).acc(3,:),'b')
    hold off
    k=k+1;
    subplot(2+no_of_links,2,k);
    hold on
    plot(t,P_link(2+i).angacc(1,:),'r')
    plot(t,P_link(2+i).angacc(2,:),'g')
    plot(t,P_link(2+i).angacc(3,:),'b')
    hold off
end

