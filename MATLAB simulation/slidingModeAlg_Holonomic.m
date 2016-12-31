clc;
clear;
close all;

global u;


Tsamp = 1;


X0 = [0  -20]';

G = [0 40]';
u = [0 0]';
 
L1 = [-20 80]';
L2 = [10 80]';
L3 = [20 80]';

theta_1 = atan2(L1(2)- G(2), L1(1)-G(1));
theta_2 = atan2(L2(2)- G(2), L2(1)-G(1));
theta_3 = atan2(L3(2)- G(2), L3(1)-G(1));

beta_21 = theta_2 - theta_1;
beta_21s = atan2(sin(beta_21),cos(beta_21));

beta_32 = theta_3 - theta_2;
beta_32s = atan2(sin(beta_32),cos(beta_32));

beta_13 = theta_1 - theta_3;
beta_13s = atan2(sin(beta_13),cos(beta_13));

path = zeros(2,1);
path(:,1) = X0;

for k=1:3000
    [t,X] = ode45('holModel',Tsamp, path(:,end));
    theta_1 = atan2(L1(2)- X(end,2), L1(1)- X(end,1));
    theta_2 = atan2(L2(2)- X(end,2), L2(1)- X(end,1));
    theta_3 = atan2(L3(2)- X(end,2), L3(1)- X(end,1));
    
    beta_21 = theta_2 - theta_1;
    beta_21 = atan2(sin(beta_21),cos(beta_21));
    
    beta_32 = theta_3 - theta_2;
    beta_32 = atan2(sin(beta_32),cos(beta_32));
    
    beta_13 = theta_1 - theta_3;
    beta_13 = atan2(sin(beta_13),cos(beta_13));
    
    OP1 = L1 - X(end,:)';
    OP1 = OP1./norm(OP1);
    OP2 = L2 - X(end,:)';
    OP2 = OP2./norm(OP2);
    OP3 = L3 - X(end,:)';
    OP3 = OP3./norm(OP3);
    
    M = [cos(beta_13/2)*(beta_13 - beta_13s) 0 cos(beta_13/2)*(beta_13 - beta_13s);
        cos(beta_21/2)*(beta_21 - beta_21s) cos(beta_21/2)*(beta_21 - beta_21s) 0;
        0 cos(beta_32/2)*(beta_32 - beta_32s) cos(beta_32/2)*(beta_32 - beta_32s)];
    N = [OP1 OP2 OP3]';
    
    V = 2.*M*N;
    
    u = (V(1,:) + V(2,:) +V(3,:))'
    
    path = [path X(end,:)'];
    
    if(norm(path(:,end) - G)<.1)
        break;
    end
    
end

figure(1); plot(path(1,:),path(2,:)); hold on;


