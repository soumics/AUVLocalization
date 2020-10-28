%% This function returns Mu=[f_auv,Tau_auv,Tau_1,....Tau_n]
%% T(:,1)=f_auv; Tau=[Tau_auv,Tau_1,....Tau_n]
function Mu=inv_dyn(q,qdot,qddot,plot_var)

%gg=[0;0;9.81]

%qddot(1:3)=qddot(1:3)
[T,Tau]=force_torque(q,qdot,qddot,plot_var);
Mu=[T(:,1) Tau];