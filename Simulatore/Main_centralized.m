clc;
clear all;


% Time
Dt = 1e-2;
t = 0:Dt:10;
n = 2;

% Unicycle: x,y,theta
% - Vehicle params
Unicyclea.r = 0.2; % m
Unicyclea.d = 1.5; % m
Unicyclea.idx = 1;
% - Initial condition
Unicyclea.q0 = [10; -10; 0; 20; -20; 0];
% - Inputs
Unicyclea.v = 2*ones(length(t),1);

Unicyclea.omega = sin(0.3*t);
% - State
Unicyclea.x = zeros(length(Unicyclea.q0), length(t));
Unicyclea.x(:,1) = Unicyclea.q0;
Unicyclea.x_hat = zeros(length(Unicyclea.q0), length(t));
Unicyclea.P_hat = zeros(6,6,length(t));
Unicyclea.P_hat(1:3,1:3,1) = diag([0.1/3, 0.1/3, 2*(pi/180)/3].^2);
Unicyclea.P_hat(4:6,4:6,1) = diag([0.1/3, 0.1/3, 2*(pi/180)/3].^2);
Unicyclea.x_hat(:,1) =Unicyclea.q0 + randn(6,1).*sqrt(diag(Unicyclea.P_hat(:,:,1)));
% - Measurements
Unicyclea.v_est = 0;
Unicyclea.omega_est = 0;
Unicyclea.theta_est = 0;
Unicyclea.xgps = 0;
Unicyclea.ygps = 0;
Unicyclea.dist_x = 0;
Unicyclea.dist_y = 0;
% - Sensors
Unicyclea.Enc.RW = zeros(1,length(t));
Unicyclea.Enc.LW = zeros(1,length(t));
Unicyclea.EncRW = Unicyclea.Enc.RW(1);
Unicyclea.EncLW = Unicyclea.Enc.LW(1);



Unicycleb = Unicyclea;



IMM.N = 2; %number of filters.
IMM.PI = [0.8 0.2; 0.2 0.8]; % probability Matrix
IMM.mu_hat = zeros(IMM.N,length(t));
IMM.mu_hat(:,1) = [0.8 0.2]';
IMM.c = zeros(IMM.N,length(t));
IMM.mu_tilde = zeros(IMM.N);
IMM.P_hat = zeros(9,9);
IMM.x_hat = zeros(6,length(t));
IMM.c_bar = zeros(IMM.N,length(t));

x_hat_0a = zeros(length(Unicyclea.q0), length(t));
x_hat_0b = zeros(length(Unicyclea.q0), length(t));

IMM.x_hat(:,1) = Unicyclea.x_hat(:,1)*IMM.mu_hat(1,1)+Unicycleb.x_hat(:,1)*IMM.mu_hat(2,1);
IMM.P_hat = IMM.mu_hat(1,1)*(Unicyclea.P_hat(:,:,1)+(Unicyclea.x_hat(:,1)-IMM.x_hat(:,1))*(Unicyclea.x_hat(:,1)-IMM.x_hat(:,1))')+...
            IMM.mu_hat(2,1)*(Unicycleb.P_hat(:,:,1)+(Unicycleb.x_hat(:,1)-IMM.x_hat(:,1))*(Unicycleb.x_hat(:,1)-IMM.x_hat(:,1))');
        
mse = zeros(length(Unicyclea.q0),length(t));
mse(:,1) = sqrt((IMM.x_hat(:,1)-Unicyclea.x(:,1)).^2);
mmse = mse(:,1);

for k = 1:length(t)-1
    
    
 
    gps = 0;
    if(rand(1) > 0.5)
        gps = 1;
        
    end
    
    %Update the model
    
    Unicyclea = Model(Unicyclea,k);
    Unicycleb = Model(Unicycleb,k);
  
    %Get measurements and estimation for inputs
    
    Unicyclea = Measure(Unicyclea,k);
    Unicycleb = Measure(Unicycleb,k);
      
    if(gps == 0)
        rhoa =  [Unicyclea.dist_x;...
                Unicyclea.dist_y;...
                Unicyclea.dist_t];
        rhob =  [Unicycleb.dist_x;...
                Unicycleb.dist_y;...
                Unicycleb.dist_t];
    end
    if(gps == 1) 
        rhoa =  [Unicyclea.xgps;...
                Unicyclea.ygps;...
                Unicyclea.theta_est];
        rhob =  [Unicycleb.xgps;...
                Unicycleb.ygps;...
                Unicycleb.theta_est];
    end
    
    %Prediction on mu
    
    IMM = IMMprediction(IMM,k);
    
    % State Interaction
    
    x_hat_0a(:,k+1) = Unicyclea.x_hat(:,k)*IMM.mu_tilde(1,1)+Unicycleb.x_hat(:,k)*IMM.mu_tilde(2,1);
    x_hat_0b(:,k+1) = Unicyclea.x_hat(:,k)*IMM.mu_tilde(1,2)+Unicycleb.x_hat(:,k)*IMM.mu_tilde(2,2);
    
    
    xa = Unicyclea.x_hat(:,k);
    xb = Unicycleb.x_hat(:,k);
    xa0 = x_hat_0a(:,k+1);
    xb0 = x_hat_0b(:,k+1);
    Pa = Unicyclea.P_hat(:,:,k);
    Pb = Unicycleb.P_hat(:,:,k);
    
    P_hat_0a = IMM.mu_tilde(1,1)*(Pa+(xa-xa0)*(xa-xa0)')+ ...
               IMM.mu_tilde(2,1)*(Pb+(xb-xa0)*(xb-xa0)');
    
    P_hat_0b = IMM.mu_tilde(1,2)*(Pa+(xa-xb0)*(xa-xb0)')+ ...
               IMM.mu_tilde(2,2)*(Pb+(xb-xb0)*(xb-xb0)');
    
    %if(mod(k,50) == 0)
    xa = x_hat_0a(:,k+1);%Unicyclea.x_hat(:,k);%
    Pa = P_hat_0a;%Unicyclea.P_hat(:,:,k);%
    Za = rhoa;
    ua = [Unicyclea.v_est,Unicyclea.omega_est];
    %else
%     xa = Unicyclea.x_hat(:,k);%x_hat_0a(:,k+1);%
%     Pa = Unicyclea.P_hat(:,:,k);%P_hat_0a;%
%     Za = rhoa;
%     ua = [Unicyclea.v_est,Unicyclea.omega_est];
    %end
    
    if(gps==1)
        [x_plus, P_plus,Sa,ra] = EKF_split(xa,Pa,Za,ua,"a");
        Unicyclea.x_hat(:,k+1) = x_plus;
        Unicyclea.P_hat(:,:,k+1) = P_plus;
    elseif(gps==0)
        [x_plus,P_plus,Sa,ra] = EKF_split_relative(xa,Pa,Za,ua,"a");
        Unicyclea.x_hat(:,k+1) = x_plus;
        Unicyclea.P_hat(:,:,k+1) = P_plus;
    end
    
     
    
   % if(mod(k,50) == 0)
        
    xb = x_hat_0b(:,k+1);%Unicycleb.x_hat(:,k);
    Pb = P_hat_0b;%Unicycleb.P_hat(:,:,k);
    Zb = rhob;
    ub = [Unicycleb.v_est,Unicycleb.omega_est];
%     else
%     xb = Unicycleb.x_hat(:,k);%x_hat_0b(:,k+1);%
%     Pb = Unicycleb.P_hat(:,:,k);%P_hat_0b;%
%     Zb = rhob;
%     ub = [Unicycleb.v_est,Unicycleb.omega_est];
%     end
    
    if(gps==1)
        [x_plus, P_plus,Sb,rb] = EKF_split(xb,Pb,Zb,ub,"b");
        Unicycleb.x_hat(:,k+1) = x_plus;
        Unicycleb.P_hat(:,:,k+1) = P_plus;
    elseif(gps==0)
        [x_plus,P_plus,Sb,rb] = EKF_split_relative(xb,Pb,Zb,ub,"b");
        Unicycleb.x_hat(:,k+1) = x_plus;
        Unicycleb.P_hat(:,:,k+1) = P_plus;
    end
    
    

    %Probability update

    IMM = IMMupdate(IMM,Sa,Sb,ra,rb,k);
        
    
    IMM.x_hat(:,k+1) = Unicyclea.x_hat(:,k+1)*IMM.mu_hat(1,k+1)+Unicycleb.x_hat(:,k+1)*IMM.mu_hat(2,k+1);
    IMM.P_hat = IMM.mu_hat(1,k+1)*(Unicyclea.P_hat(:,:,k+1)+(Unicyclea.x_hat(:,k+1)-IMM.x_hat(:,k+1))*(Unicyclea.x_hat(:,k+1)-IMM.x_hat(:,k+1))')+...
                IMM.mu_hat(2,k+1)*(Unicycleb.P_hat(:,:,k+1)+(Unicycleb.x_hat(:,k+1)-IMM.x_hat(:,k+1))*(Unicycleb.x_hat(:,k+1)-IMM.x_hat(:,k+1))');
        
    
    mse(:,k+1) = sqrt((IMM.x_hat(:,k+1)-Unicyclea.x(:,k+1)).^2);
    mmse = mmse+mse(:,k+1);
end
mmse = mmse/(k+1);
temp(:,1:length(t)) = mmse + zeros(6,length(t));
figure(1), clf, hold on;
plot(Unicyclea.x(1,:),  Unicyclea.x(2,:), 'b');
plot(Unicyclea.x_hat(1,:), Unicyclea.x_hat(2,:), 'r');

title('model 1-agent 1 trajectory');
legend('x','x_{hat}');
xlabel('[m]');
ylabel('[m]');

figure(2), clf, hold on;
plot(Unicyclea.x(4,:),  Unicyclea.x(5,:), 'b');
plot(Unicyclea.x_hat(4,:), Unicyclea.x_hat(5,:), 'r');
title('model 1-agent 2 trajectory');
legend('x','x_{hat}')
xlabel('[m]');
ylabel('[m]');

figure(3), clf, hold on;
plot(Unicycleb.x(1,:),  Unicycleb.x(2,:), 'b');
plot(Unicycleb.x_hat(1,:), Unicycleb.x_hat(2,:), 'r');
title('model 2 - agent 1 trajectory');
legend('x','x_{hat}')
xlabel('[m]');
ylabel('[m]');


figure(4), clf, hold on;
plot(Unicycleb.x(4,:),  Unicycleb.x(5,:), 'b');
plot(Unicycleb.x_hat(4,:), Unicycleb.x_hat(5,:), 'r');
title('model 2 - agent 2 trajectory');
legend('x','x_{hat}')
xlabel('[m]');
ylabel('[m]');


figure(5), clf, hold on;
plot(Unicyclea.x(1,:),  Unicyclea.x(2,:), 'b');
plot(IMM.x_hat(1,:), IMM.x_hat(2,:), 'r');
title('IMM trajectory');
xlabel('[m]');
ylabel('[m]');

figure(6), clf, hold on;
plot(t,IMM.mu_hat(1,:), 'b');
plot(t,IMM.mu_hat(2,:), 'r');
title('IMM probabilities');
legend('model 1','model 2')
axis([0 10 -0.1 1.1]);


figure(7), clf, hold on;
plot(t,mse(1,:), 'b');
plot(t,temp(1,:), 'r');
title('Centralized IMM X coord error');
axis([0 10 0 0.15]);

figure(8), clf, hold on;
plot(t,mse(2,:), 'b');
plot(t,temp(2,:), 'r');
axis([0 10 0 0.15]);
title('Centralized IMM Y coord error');

figure(9), clf, hold on;
plot(t,mse(3,:), 'b');
plot(t,temp(3,:), 'r');
title('Centralized IMM \theta coord error');
axis([0 10 0 0.035]);

figure(10), clf, hold on;
plot(t,  Unicyclea.x(1,:), 'b');
plot(t, Unicyclea.x_hat(1,:), 'r');
plot(t, Unicycleb.x_hat(1,:), 'g');
title('X Coordinate');
legend('x','x_{hat} model 1','x_{hat} model 2')
xlabel('[t]');
ylabel('[m]');

figure(11), clf, hold on;
plot(t, Unicyclea.x(2,:), 'b');
plot(t, Unicyclea.x_hat(2,:), 'r');
plot(t, Unicycleb.x_hat(2,:), 'g');
title('Y Coordinate');
legend('y','y_{hat} model 1','y_{hat} model 2')
xlabel('[t]');
ylabel('[m]');

figure(12), clf, hold on;
plot(t, Unicyclea.x(3,:), 'b');
plot(t, Unicyclea.x_hat(3,:), 'r');
plot(t, Unicycleb.x_hat(3,:), 'g');
title('\Theta Coordinate');
legend('\theta','\theta_{hat} model 1','\theta_{hat} model 2')
xlabel('[t]');
ylabel('[m]');


figure(13), clf, hold on;
plot(Unicyclea.x(1,:), Unicyclea.x(2,:), 'b');
plot(IMM.x_hat(1,:),IMM.x_hat(2,:), 'r');
title('IMM Trajectory');
legend('Robot trajectory','Predicted IMM trajectory')
xlabel('[t]');
ylabel('[m]');