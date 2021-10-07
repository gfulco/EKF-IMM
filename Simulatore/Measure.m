function [Unicycle] = Measure(Unicycle,k)
%UNTITLED7 Summary of this function goes here
Dt = 1e-2;

Sensor.Enc.Ticks = 360;
Sensor.Enc.Quanta = 2*pi/Sensor.Enc.Ticks;
Sensor.Camera.Sigma = [0.1/3, 0.1/3, 2*(pi/180)/3];
Sensor.GPS.Sigma = [0.5/3, 0.5/3 2*(pi/180)/3];
Sensor.Range.Sigma = [0.2/3, 0.1/3, 2*(pi/180)/3]';

    OmegaR = Unicycle.v(k)/Unicycle.r + Unicycle.omega(k)*Unicycle.d/(2*Unicycle.r);
    OmegaL = Unicycle.v(k)/Unicycle.r - Unicycle.omega(k)*Unicycle.d/(2*Unicycle.r);
    Unicycle.Enc.RW(k+1) = Unicycle.Enc.RW(k) + Dt*OmegaR;
    Unicycle.Enc.LW(k+1) = Unicycle.Enc.LW(k) + Dt*OmegaL;
    EncRWk = Unicycle.EncRW;
    EncLWk = Unicycle.EncLW;
    Unicycle.EncRW = (floor(Unicycle.Enc.RW(k+1)/Sensor.Enc.Quanta) + round((rand(1)-0.5)*5))*Sensor.Enc.Quanta;
    Unicycle.EncLW = (floor(Unicycle.Enc.LW(k+1)/Sensor.Enc.Quanta) + round((rand(1)-0.5)*5))*Sensor.Enc.Quanta;
    Unicycle.Enc.RW(k+1) = Unicycle.EncRW;
    Unicycle.Enc.LW(k+1) = Unicycle.EncLW;
    Unicycle.v_est = ((Unicycle.EncRW - EncRWk) + (Unicycle.EncLW - EncLWk))*Unicycle.r/2;
    Unicycle.omega_est = ((Unicycle.EncRW - EncRWk) - (Unicycle.EncLW - EncLWk))*Unicycle.r/Unicycle.d;
    
    
    % - Camera readings
    Unicycle.theta_est = Unicycle.x(3,k+1) + Sensor.Camera.Sigma(3)*randn(1);
    % - GPS
    Unicycle.xgps = Unicycle.x(1,k+1) + Sensor.GPS.Sigma(1)*randn(1);
    Unicycle.ygps = Unicycle.x(2,k+1) + Sensor.GPS.Sigma(2)*randn(1);

    % - Ranging system
    Unicycle.dist_x = Unicycle.x(4,k+1) - Unicycle.x(1,k+1) + Sensor.Range.Sigma(1)*randn(1);
    Unicycle.dist_y = Unicycle.x(5,k+1) - Unicycle.x(2,k+1) + Sensor.Range.Sigma(2)*randn(1);    
    Unicycle.dist_t = Unicycle.x(6,k+1) - Unicycle.x(3,k+1) + Sensor.Range.Sigma(3)*randn(1);
end

