%% This function returns the S matrix (cross product operator matrix) with lambda=[lambda_1,lambda_2,lambda_3] passed as arguments 
function S=Smtrx(lambda)
S=[0 -lambda(3) lambda(2);
    lambda(3) 0 -lambda(1);
    -lambda(2) lambda(1) 0];