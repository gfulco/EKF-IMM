clear all;
clc;

% Time
Dt = 1e-2;
t = 0:Dt:10;

% Unicycle: x,y,theta
% - Initial condition
q0 = [10; -10; pi];
% - Inputs
v = sin(0.2*t)*2;
omega = cos(0.1*t).*sin(0.3*t);
% - State
q = zeros(length(q0), length(t));
q(:,1) = q0;
q_est = zeros(length(q0), length(t));
Pk = diag([0.1/3, 0.1/3, 2*(pi/180)/3].^2);
q_est(:,1) = q0 + randn(3,1).*sqrt(diag(Pk));
% - Sensors
Sensor.Enc.RW = zeros(1,length(t));
Sensor.Enc.LW = zeros(1,length(t));
EncRW = Sensor.Enc.RW(1);
EncLW = Sensor.Enc.LW(1);
Sensor.Enc.Ticks = 360;
Sensor.Enc.Quanta = 2*pi/Sensor.Enc.Ticks;
Sensor.Camera.Sigma = [0.1/3, 0.1/3, 2*(pi/180)/3];
Sensor.GPS.Sigma = [0.5/3, 0.5/3];
Sensor.Range.Sigma = [0.2/3, 0.1/3, 0.3/3]';
Sensor.Range.Pos = [0, 0; 0, 10; 15, 0];
% - Vehicle params
r = 0.2; % m
d = 1.5; % m

for k = 1:length(t)-1
    
    % Unicycle kinematic model
    xk = q(1,k);
    yk = q(2,k);
    thetak = q(3,k);
    % Next step
    xk_1 = xk + cos(thetak)*v(k)*Dt;
    yk_1 = yk + sin(thetak)*v(k)*Dt;
    thetak_1 = thetak + omega(k)*Dt;
    % Storing
    q(:,k+1) = [xk_1; yk_1; thetak_1];
    
    % Measurements
    % - Encoders
    OmegaR = v(k)/r + omega(k)*d/(2*r);
    OmegaL = v(k)/r - omega(k)*d/(2*r);
    Sensor.Enc.RW(k+1) = Sensor.Enc.RW(k) + Dt*OmegaR;
    Sensor.Enc.LW(k+1) = Sensor.Enc.LW(k) + Dt*OmegaL;
    EncRWk = EncRW;
    EncLWk = EncLW;
    EncRW = (floor(Sensor.Enc.RW(k+1)/Sensor.Enc.Quanta) + round((rand(1)-0.5)*5))*Sensor.Enc.Quanta;
    EncLW = (floor(Sensor.Enc.LW(k+1)/Sensor.Enc.Quanta) + round((rand(1)-0.5)*5))*Sensor.Enc.Quanta;
    Sensor.Enc.RW(k+1) = EncRW;
    Sensor.Enc.LW(k+1) = EncLW;
    v_est_Dt = ((EncRW - EncRWk) + (EncLW - EncLWk))*r/2;
    omega_est_Dt = ((EncRW - EncRWk) - (EncLW - EncLWk))*r/d;
    % - Camera readings
    xc = q(1,k+1) + Sensor.Camera.Sigma(1)*randn(1);
    yc = q(2,k+1) + Sensor.Camera.Sigma(2)*randn(1);
    thetac = q(3,k+1) + Sensor.Camera.Sigma(3)*randn(1);
    % - GPS
    xgps = q(1,k+1) + Sensor.GPS.Sigma(1)*randn(1);
    ygps = q(2,k+1) + Sensor.GPS.Sigma(2)*randn(1);
    % - Ranging system
    rho = [sqrt((q(1,k+1) - Sensor.Range.Pos(1,1))^2 + (q(2,k+1) - Sensor.Range.Pos(1,2))^2);
        sqrt((q(1,k+1) - Sensor.Range.Pos(2,1))^2 + (q(2,k+1) - Sensor.Range.Pos(2,2))^2);
        sqrt((q(1,k+1) - Sensor.Range.Pos(3,1))^2 + (q(2,k+1) - Sensor.Range.Pos(3,2))^2)] + ...
        randn(3,1).*Sensor.Range.Sigma;

    % Estimated model
    xk = q_est(1,k);
    yk = q_est(2,k);
    thetak = q_est(3,k);
    % Prediction
    
    % - State
    xk_1 = xk + cos(thetak)*v_est_Dt;
    yk_1 = yk + sin(thetak)*v_est_Dt;
    thetak_1 = thetak + omega_est_Dt;
    % - Covariance
    Q = diag([4*Sensor.Enc.Quanta/3; 4*Sensor.Enc.Quanta/3].^2);
    A = [1, 0, -sin(thetak)*v_est_Dt;
         0, 1,  cos(thetak)*v_est_Dt;
         0, 0, 1];
    B = [r/2*cos(thetak), r/2*cos(thetak);
         r/2*sin(thetak), r/2*sin(thetak);
         r/d, -r/d];
    Pk_1 = A*Pk*A' + B*Q*B';
    % Update
    % - Linearised output matrix
    rho_pred = [sqrt((xk_1 - Sensor.Range.Pos(1,1))^2 + (yk_1 - Sensor.Range.Pos(1,2))^2);
        sqrt((xk_1 - Sensor.Range.Pos(2,1))^2 + (yk_1 - Sensor.Range.Pos(2,2))^2);
        sqrt((xk_1 - Sensor.Range.Pos(3,1))^2 + (yk_1 - Sensor.Range.Pos(3,2))^2)];
    H = [(xk_1 - Sensor.Range.Pos(1,1))/rho_pred(1), (yk_1 - Sensor.Range.Pos(1,2))/rho_pred(1), 0;
        (xk_1 - Sensor.Range.Pos(2,1))/rho_pred(2), (yk_1 - Sensor.Range.Pos(2,2))/rho_pred(2), 0;
        (xk_1 - Sensor.Range.Pos(3,1))/rho_pred(3), (yk_1 - Sensor.Range.Pos(3,2))/rho_pred(3), 0];
    % - Covariance of the measurements
    R = diag(Sensor.Range.Sigma.^2);
    % - Measurements
    z = rho;
    % - Covariance of the innovations
    S = H*Pk_1*H' + R;
    % - Gain
    W = Pk_1*H'*inv(S);
    % - State update
    q_up = [xk_1; yk_1; thetak_1] + W*(z - rho_pred);
    % - Covariance update
    P_up = (eye(3) - W*H)*Pk_1;
    % Storing
    q_est(:,k+1) = q_up;
    Pk = P_up;
 
end

figure(1), clf, hold on;
plot(t, q(1,:), 'b');
plot(t, q_est(1,:), 'r--');
title('x');
xlabel('[s]');

figure(2), clf, hold on;
plot(t, q(2,:), 'b');
plot(t, q_est(2,:), 'r--');
title('y');
xlabel('[s]');

figure(3), clf, hold on;
plot(t, q(3,:), 'b');
plot(t, q_est(3,:), 'r--');
title('\theta');
xlabel('[s]');