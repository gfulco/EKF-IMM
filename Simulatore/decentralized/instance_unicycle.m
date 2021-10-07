function [Unicycle] = instance_unicycle(radius, diam, idx, num, steps)

% Unicycle: x,y,theta
% - Vehicle params

Unicycle.r = radius; % m
Unicycle.d = diam; % m
Unicycle.idx = idx;

% - Initial condition

Unicycle.q0 = rand(num,1)*2*pi;  %[rand(1,1)*100; rand(1,1)*100; 0; rand(1,1)*100; rand(1,1)*100; 0];

% - Inputs

Unicycle.v = 2*ones(steps,1);
Unicycle.omega = sin(0.3*steps/100);

% - State

Unicycle.x = zeros(length(Unicyclea.q0), steps);
Unicycle.x(:,1) = Unicyclea.q0;
Unicycle.x_hat = zeros(length(Unicyclea.q0));
Unicycle.P_hat = zeros(3*num,3*num,steps);
for i = 0:num-1
    Unicycle.P_hat((3*i)+1:(3*i)+3,(3*i)+1:(3*i)+3,1) = diag([0.1/3, 0.1/3, 2*(pi/180)/3].^2);
                                                                %Unicycle.P_hat(4:6,4:6) = diag([0.1/3, 0.1/3, 2*(pi/180)/3].^2,1);
end
Unicycle.x_hat(:,1) =Unicyclea.q0 + randn(3*num,1).*sqrt(diag(Unicyclea.P_hat(:,:,1)));

% - Measurements

Unicycle.v_est = 0;
Unicycle.omega_est = 0;
Unicycle.theta_est = 0;
Unicycle.xgps = 0;
Unicycle.ygps = 0;
Unicycle.dist_x = 0;
Unicycle.dist_y = 0;
% - Sensors

Unicycle.Enc.RW = zeros(1,steps);
Unicycle.Enc.LW = zeros(1,steps);
Unicycle.EncRW = Unicyclea.Enc.RW(1);
Unicycle.EncLW = Unicyclea.Enc.LW(1);


end