function plotofdata(q,qdot,qddot,samples,t)

global no_of_links;

POSITION_PLOT=1;
EULARANG_PLOT=1;
VELOCITY_PLOT=0;
ACCELERATION_PLOT=0;
FORCE_PLOT=0;
SAVE_FIG=1;

    for i=1:samples
        [P R V W Vd Wd]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
        
        P_link(1).pos(:,i)=P(:,1);
        P_link(2).pos(:,i)=P(:,2);
        P_link(1).vel(:,i)=V(:,1);
        P_link(1).angvel(:,i)=W(:,1);
        P_link(2).vel(:,i)=V(:,2);
        P_link(2).angvel(:,i)=W(:,2);
        P_link(1).acc(:,i)=Vd(:,1);
        P_link(1).angacc(:,i)=Wd(:,1);
        P_link(2).acc(:,i)=Vd(:,2);
        P_link(2).angacc(:,i)=Wd(:,2);
        for j=1:no_of_links
            P_link(2+j).pos(:,i)=P(:,2+j);
            P_link(2+j).vel(:,i)=V(:,2+j);
            P_link(2+j).angvel(:,i)=W(:,2+j);
            P_link(2+j).acc(:,i)=Vd(:,2+j);
            P_link(2+j).angacc(:,i)=Wd(:,2+j);
        end
    end
    


if POSITION_PLOT==1
    
%     for i=1:samples
%         [a b]=pos_rot_pnts(q(:,i));
%         %pos_P(:,i)=a(:,1);
%         P_link(1).pos(:,i)=a(:,1);
%         P_link(2).pos(:,i)=a(:,2);
%         
%         for j=1:no_of_links
%             P_link(2+j).pos(:,i)=a(:,2+j);
%         end
%     end
    
    figure
    thisax = subplot(2,1,1);
    
    hold on
    plot(t,P_link(1).pos(1,:),'r')
    plot(t,P_link(1).pos(2,:),'g')
    plot(t,P_link(1).pos(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV Centroid - XYZ [m]');
    
    hold off
    
    thisax = subplot(2,1,2)
    hold on
    plot(t,P_link(2).pos(1,:),'r')
    plot(t,P_link(2).pos(2,:),'g')
    plot(t,P_link(2).pos(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV Attachment Point P [m]');
    hold off
    
    if SAVE_FIG==1
        cd img
        saveas(gcf,'Pos_auv.jpg')
        cd ..
    end
    
    for i=1:no_of_links
        figure
        %thisax=subplot(2+no_of_links,1,2+i)
        hold on
        
        plot(t,P_link(2+i).pos(1,:),'r')
        plot(t,P_link(2+i).pos(2,:),'g')
        plot(t,P_link(2+i).pos(3,:),'b')
        legend('X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Link ' num2str(i) ' positions [m]']);
        hold off
        if SAVE_FIG==1
            cd img
            saveas(gcf,['Pos_link' num2str(i) '.jpg'])
            cd ..
        end
        
    end
    %saveas(gcf,'Positions.jpg')
    
end

if EULARANG_PLOT==1
    
    figure
    
    
    hold on
    
    plot(t,q(4,1:length(q(4,:))-1),'r')
    plot(t,q(5,1:length(q(5,:))-1),'g')
    plot(t,q(6,1:length(q(6,:))-1),'b')
    legend('roll','pitch','yaw', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV r-p-y [rad]');
    
    hold off
    if SAVE_FIG==1
        cd img
        saveas(gcf,'EularAng_auv.jpg')
        cd ..
    end
    
    for i=1:no_of_links
        figure
        hold on
        q_var=q(7+3*(i-1):6+3*i,:);
        plot(t,q_var(1,1:length(q_var(1,:))-1),'r')
        plot(t,q_var(2,1:length(q_var(2,:))-1),'g')
        plot(t,q_var(3,1:length(q_var(3,:))-1),'b')
        legend('roll','pitch','yaw', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Link ' num2str(i) ' r-p-y [rad]']);
        hold off
        if SAVE_FIG==1
            cd img
            saveas(gcf,['EularAng_link' num2str(i) '.jpg'])
            cd ..
        end
    end
    %saveas(gcf,'EularAng_auv.jpg')
    
end

if VELOCITY_PLOT==1
    
%     for i=2:samples
%         [na1 na2 a1 b1 na3 na4]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
%         %     vel_V(:,i)=a1(:,1);
%         %     angvel_W(:,i)=b1(:,1);
%         P_link(1).vel(:,i)=a1(:,1);
%         P_link(1).angvel(:,i)=b1(:,1);
%         P_link(2).vel(:,i)=a1(:,2);
%         P_link(2).angvel(:,i)=b1(:,2);
%         
%         for j=1:no_of_links
%             P_link(2+j).vel(:,i)=a1(:,2+j);
%             P_link(2+j).angvel(:,i)=b1(:,2+j);
%             
%         end
%     end
    
    figure
    thisax=subplot(2,2,1);
    hold on
    
    plot(t,P_link(1).vel(1,:),'r')
    plot(t,P_link(1).vel(2,:),'g')
    plot(t,P_link(1).vel(3,:),'b')
    
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV Linear Velocity [m/s]');
    hold off
    
    thisax=subplot(2,2,2);
    hold on
    plot(t,P_link(1).angvel(1,:),'r')
    plot(t,P_link(1).angvel(2,:),'g')
    plot(t,P_link(1).angvel(3,:),'b')
    
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV Angular Velocity [rad/s]');
    hold off
    
    thisax=subplot(2,2,3);
    hold on
    
    plot(t,P_link(2).vel(1,:),'r')
    plot(t,P_link(2).vel(2,:),'g')
    plot(t,P_link(2).vel(3,:),'b')
    
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Velocity of P [m/s]');
    hold off
    
    thisax=subplot(2,2,4);
    hold on
    plot(t,P_link(2).angvel(1,:),'r')
    plot(t,P_link(2).angvel(2,:),'g')
    plot(t,P_link(2).angvel(3,:),'b')
    
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Angular velocity of P [rad/s]');
    hold off
    if SAVE_FIG==1
        cd img
        saveas(gcf,'Velocities_auv.jpg')
        cd ..
    end
    
    
    for i=1:no_of_links
        figure
        thisax=subplot(1,2,1);
        hold on
        
        plot(t,P_link(2+i).vel(1,:),'r')
        plot(t,P_link(2+i).vel(2,:),'g')
        plot(t,P_link(2+i).vel(3,:),'b')
        
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Link velocity ' num2str(i) ' [m/s]']);
        hold off
        
        thisax=subplot(1,2,2);
        hold on
        plot(t,P_link(2+i).angvel(1,:),'r')
        plot(t,P_link(2+i).angvel(2,:),'g')
        plot(t,P_link(2+i).angvel(3,:),'b')
        
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Link ' num2str(i) ' angular velocity [rad/s]']);
        hold off
        if SAVE_FIG==1
            cd img
            saveas(gcf,['Velocities_link' num2str(i) '.jpg'])
            cd ..
        end
        %saveas(gcf,'Velocities.jpg')
        
    end
end
if ACCELERATION_PLOT==1
    
%     for i=2:samples
%         [na1 na2 na3 na4 a2 b2]=acc_angacc(q(:,i),qdot(:,i),qddot(:,i));
%         
%         %     acc_Vd(:,i)=a2(:,1);
%         %     angacc_Wd(:,i)=b2(:,1);
%         
%         P_link(1).acc(:,i)=a2(:,1);
%         P_link(1).angacc(:,i)=b2(:,1);
%         P_link(2).acc(:,i)=a2(:,2);
%         P_link(2).angacc(:,i)=b2(:,2);
%         
%         for j=1:no_of_links
%             P_link(2+j).acc(:,i)=a2(:,2+j);
%             P_link(2+j).angacc(:,i)=b2(:,2+j);
%             
%         end
%     end
    
    figure
    thisax=subplot(2,2,1);
    hold on
    
    plot(t,P_link(1).acc(1,:),'r')
    plot(t,P_link(1).acc(2,:),'g')
    plot(t,P_link(1).acc(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV linear acceleration [m/s^2]');
    hold off
    
    thisax=subplot(2,2,2);
    hold on
    plot(t,P_link(1).angacc(1,:),'r')
    plot(t,P_link(1).angacc(2,:),'g')
    plot(t,P_link(1).angacc(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('AUV angular acceleration [rad/s^2]');
    hold off
    
    thisax=subplot(2,2,3);
    hold on
    
    plot(t,P_link(2).acc(1,:),'r')
    plot(t,P_link(2).acc(2,:),'g')
    plot(t,P_link(2).acc(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Linear acceleration of P [m/s^2]');
    hold off
    
    thisax=subplot(2,2,4);
    hold on
    plot(t,P_link(2).angacc(1,:),'r')
    plot(t,P_link(2).angacc(2,:),'g')
    plot(t,P_link(2).angacc(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Angular acceleration of P [rad/s^2]');
    hold off
    if SAVE_FIG==1
        cd img
        saveas(gcf,'Accelerations_auv.jpg')
        cd ..
    end
    
    
    for i=1:no_of_links
        figure
        thisax=subplot(1,2,1);
        hold on
        
        plot(t,P_link(2+i).acc(1,:),'r')
        plot(t,P_link(2+i).acc(2,:),'g')
        plot(t,P_link(2+i).acc(3,:),'b')
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Linear acceleration of Link ' num2str(i) ' [m/s^2]']);
        hold off
        
        thisax=subplot(1,2,2);
        hold on
        plot(t,P_link(2+i).angacc(1,:),'r')
        plot(t,P_link(2+i).angacc(2,:),'g')
        plot(t,P_link(2+i).angacc(3,:),'b')
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Angular acceleration of Link ' num2str(i) ' [rad/s^2]']);
        hold off
        if SAVE_FIG==1
            cd img
            saveas(gcf,['Accelerations_link' num2str(i) '.jpg'])
            cd ..
        end
    end
    if SAVE_FIG==1
        cd img
        saveas(gcf,'Accelerations.jpg')
        cd ..
    end
end

if FORCE_PLOT==1
    
    for i=2:samples
        
        [a3 b3]=force_torque(q(:,i),qdot(:,i),qddot(:,i),0);
        
        %     acc_Vd(:,i)=a2(:,1);
        %     angacc_Wd(:,i)=b2(:,1);
        
        P_link(1).force(:,i)=a3(:,1);
        P_link(1).torque(:,i)=b3(:,1);
        %     P_link(2).force(:,i)=a3(:,2);
        %     P_link(2).torque(:,i)=b3(:,2);
        
        for j=1:no_of_links
            P_link(1+j).force(:,i)=a3(:,1+j);
            P_link(1+j).torque(:,i)=b3(:,1+j);
            
        end
        
    end
    
    figure
    thisax=subplot(1,2,1);
    hold on
    
    plot(t,P_link(1).force(1,:),'r')
    plot(t,P_link(1).force(2,:),'g')
    plot(t,P_link(1).force(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Forces on AUV [N]');
    hold off
    
    thisax=subplot(1,2,2);
    hold on
    plot(t,P_link(1).torque(1,:),'r')
    plot(t,P_link(1).torque(2,:),'g')
    plot(t,P_link(1).torque(3,:),'b')
    legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
    xlabel('Time [s]');
    ylabel('Torques on AUV [N-m]');
    hold off
    if SAVE_FIG==1
        cd img
        saveas(gcf,'forces_auv.jpg')
        cd ..
    end
    
    
    
    for i=1:no_of_links
        figure
        thisax=subplot(1,2,1);
        hold on
        
        plot(t,P_link(1+i).force(1,:),'r')
        plot(t,P_link(1+i).force(2,:),'g')
        plot(t,P_link(1+i).force(3,:),'b')
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Forces on Link ' num2str(i) ' [N]']);
        hold off
        
        thisax=subplot(1,2,2);
        hold on
        plot(t,P_link(1+i).torque(1,:),'r')
        plot(t,P_link(1+i).torque(2,:),'g')
        plot(t,P_link(1+i).torque(3,:),'b')
        legend(thisax,'X-dir','Y-dir','Z-dir', 'location', 'northeast')
        xlabel('Time [s]');
        ylabel(['Torques on Link ' num2str(i) ' [N-m]']);
        hold off
        if SAVE_FIG==1
            cd img
            saveas(gcf,['forces_link' num2str(i) '.jpg'])
            cd ..
        end
    end
    %saveas(gcf,'forces.jpg')
    
end
