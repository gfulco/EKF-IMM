function [x_plus,P_plus,S,r] = EKF_split(x,P,Z,u,j)
%EKF Implementation.

%datas
r = .2;
d = 1.5;

Quanta = 2*pi/360;
Sigma = [0.2/3, 0.1/3, 2*(pi/180)/3]';
%Covariance
Q = diag([4*Quanta/3; 4*Quanta/3].^2);

A1 = zeros(3,3);
B1 = zeros(3,2);
x_pred1 = zeros(3,1);
H1 = zeros(3,3);
% - Covariance of the measurements
R1 = diag(Sigma.^2);

A2 = zeros(3,3);
B2 = zeros(3,2);
x_pred2 = zeros(3,1);
H2 = zeros(3,3);
% - Covariance of the measurements

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
    x_pred1(1,1) = xk1 + cos(thetak1)*u(1);
    x_pred1(2,1) = yk1 + sin(thetak1)*u(1);
    x_pred1(3,1) = thetak1 + u(2);
    
    x_pred2(1,1) = xk2 + cos(thetak2)*u(1);
    x_pred2(2,1) = yk2 + sin(thetak2)*u(1);
    x_pred2(3,1) = thetak2 + u(2);
    elseif(strcmp(j,"b"))
    x_pred1(1,1) = xk1+20 + cos(thetak1)*u(1);
    x_pred1(2,1) = yk1+20 + sin(thetak1)*u(1);
    x_pred1(3,1) = thetak1 + 0;
    
    x_pred2(1,1) = xk2+50 + cos(thetak2)*u(1);
    x_pred2(2,1) = yk2+50 + sin(thetak2)*u(1);
    x_pred2(3,1) = thetak2 + 0;
    end
    A1(1:3,1:3) = [1, 0, -sin(thetak1)*u(1);
                  0, 1,  cos(thetak1)*u(1);
                  0, 0, 1];
              
    B1(1:3,:) = [r/2*cos(thetak1), r/2*cos(thetak1);
                r/2*sin(thetak1), r/2*sin(thetak1);
                r/d, -r/d];
    
    A2(1:3,1:3) = [1, 0, -sin(thetak2)*u(1);
                  0, 1,  cos(thetak2)*u(1);
                  0, 0, 1];
    B2(1:3,:) = [r/2*cos(thetak2), r/2*cos(thetak2);
         r/2*sin(thetak2), r/2*sin(thetak2);
         r/d, -r/d];
     
    % P matrix
    P1 = P(1:3,1:3);
    P2 = P(4:6,4:6);
    P12 = P(1:3,4:6);
     
     
    P_pred1 = A1*P1*A1' + B1*Q*B1';
    P_pred2 = A2*P2*A2' + B2*Q*B2';
    P_pred12 = A1*P12*A2';
    
    
    rho_pred = [x_pred1(1);
                x_pred1(2);
                x_pred1(3);
                ];        
 
    H1(1:3,1:3) = eye(3,3);
    H2(1:3,1:3) = zeros(3,3);
    
    
    % - Covariance of the innovations
    S = R1 + H1*P_pred1*H1'+H2*P_pred2*H2'-H2*P_pred12'*H1'-H1*P_pred12*H2';
    % - Gain
    W1 = (P_pred1*H1')*S^(-1);
    W2 = (P_pred12'*H1')*S^(-1);
    % - State update
    r = Z-rho_pred;
    
    q_up1 = x_pred1 + W1*r;
    q_up2 = x_pred2 + W2*r;
    % - Covariance update
    P_up1 = P_pred1 - W1*S*W1';
    P_up2 = P_pred2 - W2*S*W2';
    P_up12 = P_pred12 - W1*S*W2';
    % Storing
    x_plus = [q_up1;q_up2];
    P_plus = [P_up1,P_up12;
             P_up12',P_up2];
end
