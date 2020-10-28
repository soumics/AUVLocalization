%% This function adds measurement noise to the sensor output
function y=op_noise(q,qdot,iteration)

global var_usbl;
global var_dvl;
global var_acc;
global var_gyro;
global var_depth;

global var_releuang;
global var_releurate;

global sig_usbl;
global sig_depth;
global sig_gyro;

global sig_dvlx;
global sig_dvly;
global sig_dvlz;
global sig_dvlphi;
global sig_dvlth;
global sig_dvlpsi;

global sig_rng;

global sig_releuang;
global sig_releurate;

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
        n_usbl=sig_usbl^2;
        y=[y;q(1:3)+sqrt(n_usbl)*randn(3,1)];
    end
end
%depth sensor
if var_depth==1
    n_depth=sig_depth^2;
    y=[y;q(3)+sqrt(n_depth)*randn(1,1)];
    
    
end
%gyroscope
if var_gyro==1
    n_gyro=sig_gyro^2;
    y=[y;q(4:6)+sqrt(n_gyro)*randn(3,1)];
    
    
end
% doppler velocity log
if var_dvl==1
    
    n_x=sig_dvlx^2;
    n_y=sig_dvly^2;
    n_z=sig_dvlz^2;
    n_xyz=blkdiag(n_x,n_y,n_z);
    
    ydvl=qdot(1:3)+sqrt(n_xyz)*randn(3,1);
    
    y=[y;ydvl];
    
end
% accelerometer
if var_acc==1
    
    n_phi=sig_dvlphi^2;
    n_th=sig_dvlth^2;
    n_psi=sig_dvlpsi^2;
    
    yacc=qdot(4:6)+[sqrt(n_phi)*randn(1,1);sqrt(n_th)*randn(1,1);sqrt(n_psi)*randn(1,1)];
    y=[y;yacc];
end


% rabges from sparkers
if no_of_sprkrs~=0
    for j=1:no_of_sprkrs
        if SIMULATION==1
            % building the noise matrix
            n_rng=sig_rng^2;
            % receiving sensor output/building output matrix
            for k=1:no_of_links
                y=[y;norm(P(:,2+k)-XD(:,j))+sqrt(n_rng)*randn(1,1)];
            end
        else
            for k=1:no_of_links
                if ~isempty(find(range{k}.time==t_tmp))
                    t_tmp
                    if range{k}.stat_m(find(range{k}.time==t_tmp))<1000
                        y=[y;range{k}.stat_m(find(range{k}.time==t_tmp))'];
                        disp('k,t_tmp,find(range{k}.time==t_tmp)')
                        k, t_tmp
                        find(range{k}.time==t_tmp)
                        disp('range')
                        range{k}.stat_m(find(range{k}.time==t_tmp))
                    end
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
        n_releuang=sig_releuang^2;
        q_temp(i,:)=q(4+3*i)+sqrt(n_releuang)*randn(1,1);
    end
    yreleuang=q_temp;
    y=[y;yreleuang];
end
% relative roll rate
if var_releurate==1
    for i=1:no_of_links
        n_releurate=sig_releurate^2;
        qdot_temp(i,:)=qdot(4+3*i)+sqrt(n_releurate)*randn(1,1);
    end
    yreleurate=qdot_temp;
    y=[y;yreleurate];
end

% if var_releuang==1
%     n_releuang=sig_releuang^2;
%     yreleuang=q(7:6+3*no_of_links)+sqrt(n_releuang)*randn(3*no_of_links,1);
%     y=[y;yreleuang];
% end
% if var_releurate==1
%     n_releurate=sig_releurate^2;
%     yreleurate=qdot(7:6+3*no_of_links)+sqrt(n_releurate)*randn(3*no_of_links,1);
%     y=[y;yreleurate];
% end


