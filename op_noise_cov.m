%% This function adds measurement noise to the sensor output
function R=op_noise_cov(q,iteration)

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


R=[];

% usbl
if var_usbl==1
    if mod(iteration,10)==0
        
        R=blkdiag(R,sig_usbl^2);
        R=blkdiag(R,sig_usbl^2);
        R=blkdiag(R,sig_usbl^2);
    end
end
% depth sensor
if var_depth==1
    R=blkdiag(R,sig_depth^2);
end
% gyroscope
if var_gyro==1
    R=blkdiag(R,sig_gyro^2);
    R=blkdiag(R,sig_gyro^2);
    R=blkdiag(R,sig_gyro^2);
end
% doppler velocity log
if var_dvl==1
    
    n_x=sig_dvlx^2;
    n_y=sig_dvly^2;
    n_z=sig_dvlz^2;
    n_xyz=blkdiag(n_x,n_y,n_z);
    n_1=Rot_tot(q(4:6))*n_xyz*Rot_tot(q(4:6))';
    
    R=blkdiag(R,n_1);
    
end
% accelerometer
if var_acc==1
    
    n_phi=sig_dvlphi^2;
    n_th=sig_dvlth^2;
    n_psi=sig_dvlpsi^2;
    
    R=blkdiag(R,n_phi);
    R=blkdiag(R,n_th);
    R=blkdiag(R,n_psi);
    
end

%size(R)
% ranges from sparkers
if no_of_sprkrs~=0
    for j=1:no_of_sprkrs
        if SIMULATION==1
            % receiving sensor output/building output matrix
            for k=1:no_of_links
                R=blkdiag(R,sig_rng^2);
            end
        else
            for k=1:no_of_links
                if ~isempty(find(range{k}.time==t_tmp))
                    if range{k}.stat_m(find(range{k}.time==t_tmp))<1000
                        R=blkdiag(R,sig_rng^2);
                    end
                end
            end
        end
    end
end

%size(R)
% relative roll
if var_releuang==1
    for i=1:no_of_links
        R=blkdiag(R,sig_releuang^2);
    end
end
% relative roll rate
if var_releurate==1
    for i=1:no_of_links
        R=blkdiag(R,sig_releurate^2);
    end
end

% if var_releuang==1
%     for i=1:3*no_of_links
%         R=blkdiag(R,sig_releuang^2);
%     end
% end
% if var_releurate==1
%     for i=1:3*no_of_links
%         R=blkdiag(R,sig_releurate^2);
%     end
% end

