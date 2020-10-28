%% This function returns accelerations of the states given the states,
%% derivative of the states and the forces and moments

%% Function arguments:
%% q=states (auv position, orientation,eular angles
%% qdot=derivative of q
%% Tau=input forces and moments

%% Function Returns:
%% qddot=acceleration of the states

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function qddot=dir_dyn(q,qdot,Tau)
global no_of_links;
global buoyancy_max_auv;
global g;
global m_auv;

% Sets the acceleration to ZERO [vector]
qddot_links=kron(ones(no_of_links,1),[0;0;0]);
qddot_auv=zeros(6,1);
qddot=[qddot_auv;qddot_links];



% Calculation of Tau_bar

Tau_bar_temp=inv_dyn(q,qdot,qddot,0);
[row,col]=size(Tau_bar_temp);
Tau_bar=[];
for ii=1:col
    Tau_bar=[Tau_bar;Tau_bar_temp(:,ii)];
end


% Setting g, and velocity of the states to ZERO [vector]
g=[0;0;0];



% Calculation of the columns of the mass matrix

for i=1:6+3*no_of_links
    
    qdot_temp=zeros(6+3*no_of_links,1);
    qddot_temp=zeros(6+3*no_of_links,1);
    qddot_temp(i)=1;
    
    M_curr_temp=inv_dyn(q,qdot_temp,qddot_temp,0);
    [row1,col1]=size(M_curr_temp);
    
    M_temp=[];
    for j=1:col1
        M_temp=[M_temp;M_curr_temp(:,j)];
    end
    M(:,i)=M_temp;
end

% restoring g to its original value
g=[0;0;9.81];

% Calculation of the acceleration with mass matrix, Tau_bar(coriolis force
% etc. ), and given Tau (input)

qddot=inv(M)*(Tau-Tau_bar);
