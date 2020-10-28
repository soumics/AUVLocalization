%% This function adds measurement noise to the sensor output
function y=op_func(q,qdot,iteration)

global var_usbl;
global var_dvl;
global var_acc;
global var_gyro;
global var_depth;

global var_releuang;
global var_releurate;


global XD;

global no_of_links;

global t_tmp;

global range;
global SIMULATION;

[row,no_of_sprkrs]=size(XD);

[P,R]=pos_rot_pnts(q);



y=[];

% usbl
if var_usbl==1
    if mod(iteration,10)==0        
    y=[y;q(1:3)];
    end
end
% depth sensor
if var_depth==1
    
    y=[y;q(3)];
    
end
%gyroscope
if var_gyro==1
    
    y=[y;q(4:6)];
    
end
% doppler velocity log
if var_dvl==1
    
    ydvl=qdot(1:3);
    y=[y;ydvl];
    
end
% accelerometer
if var_acc==1
    
    yacc=qdot(4:6);
    y=[y;yacc];
end


%size(y)
% range from sparkers
if no_of_sprkrs~=0
    for j=1:no_of_sprkrs
        if SIMULATION==1
            % receiving sensor output/building output matrix
            for k=1:no_of_links
                y=[y;norm(P(:,2+k)-XD(:,j))];
            end
        else
            for k=1:no_of_links
                if ~isempty(find(range{k}.time==t_tmp))
                    if range{k}.stat_m(find(range{k}.time==t_tmp))<1000
                        y=[y;norm(P(:,2+k)-XD(:,j))];
                        
                    end
                    %  disp('range')
                    %  norm(P(:,2+k)-XD(:,j))
                end
            end
        end
    end
end

%size(y)
q_temp=[];
qdot_temp=[];

% relative roll
if var_releuang==1
    
    for i=1:no_of_links
        q_temp(i,:)=q(4+3*i);
    end
    yreleuang=q_temp;
    y=[y;yreleuang];
end
% relative roll rate
if var_releurate==1
    for i=1:no_of_links
        qdot_temp(i,:)=qdot(4+3*i);
    end
    yreleurate=qdot_temp;
    y=[y;yreleurate];
end

% if var_releuang==1
%     yreleuang=q(7:6+3*no_of_links);
%     y=[y;yreleuang];
% end
% if var_releurate==1
%     yreleurate=qdot(7:6+3*no_of_links);
%     y=[y;yreleurate];
% end

