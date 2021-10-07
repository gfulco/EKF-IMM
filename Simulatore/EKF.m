function [x_plus,P_plus,S,r] = EKF(x,P,Z,u,j)
%EKF Implementation.

%datas
r = .2;
d = 1.5;

Quanta = 2*pi/360;
Sigma = [0.2/3, 0.1/3, 2*(pi/180)/3]';
%Covariance
Q = diag([4*Quanta/3; 4*Quanta/3].^2);

A = zeros(6,6);
B = zeros(6,2);
x_pred = zeros(6,1);
H = zeros(3,6);
% - Covariance of the measurements
R = diag(Sigma.^2);


 % Estimated model
    xk1 = x(1);
    yk1 = x(2);
    thetak1 = x(3);
    xk2 = x(4);
    yk2 = x(5);
    thetak2 = x(6);
    % Prediction
    % - State
    if(strcmp(j,"a"))
    x_pred(1,1) = xk1 + cos(thetak1)*u(1);
    x_pred(2,1) = yk1 + sin(thetak1)*u(1);
    x_pred(3,1) = thetak1 + u(2);
    
    x_pred(4,1) = xk2 + cos(thetak2)*u(1);
    x_pred(5,1) = yk2 + sin(thetak2)*u(1);
    x_pred(6,1) = thetak2 + u(2);
    elseif(strcmp(j,"b"))
    x_pred(1,1) = xk1+20 + cos(thetak1)*u(1);
    x_pred(2,1) = yk1+20 + sin(thetak1)*u(1);
    x_pred(3,1) = thetak1 + 0;
    
    x_pred(4,1) = xk2+50 + cos(thetak2)*u(1);
    x_pred(5,1) = yk2+50 + sin(thetak2)*u(1);
    x_pred(6,1) = thetak2 + 0;
    end
    A(1:3,1:3) = [1, 0, -sin(thetak1)*u(1);
                  0, 1,  cos(thetak1)*u(1);
                  0, 0, 1];
              
    B(1:3,:) = [r/2*cos(thetak1), r/2*cos(thetak1);
                r/2*sin(thetak1), r/2*sin(thetak1);
                r/d, r/d];
    
    
    A(4:6,4:6) = [1, 0, -sin(thetak2)*u(1);
                  0, 1,  cos(thetak2)*u(1);
                  0, 0, 1];
    B(4:6,:) = [r/2*cos(thetak2), r/2*cos(thetak2);
         r/2*sin(thetak2), r/2*sin(thetak2);
         r/d, -r/d];

    P_pred = A*P*A' + B*Q*B';
    
   
    
    
    rho_pred = [x_pred(1);
                x_pred(2);
                x_pred(3);
                ];        
 
    H(1:3,1:3) = eye(3,3);
    
    
    % - Covariance of the innovations
    S = H*P_pred*H' + R;
    % - Gain
    W = P_pred*H'*S^(-1);
    % - State update
    r = Z-rho_pred;
    
    q_up = x_pred + W*r;
    % - Covariance update
    P_up = P_pred - W*H*P_pred;
    % Storing
    x_plus = q_up;
    P_plus = P_up;



end

