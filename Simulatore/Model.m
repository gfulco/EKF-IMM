function [Unicycle] = Model(Unicycle,k)
%
   %%%%MODEL UPDATE START
    % Unicycle 1 kinematic model
    
    Dt = 1e-2;
    xk = Unicycle.x(1,k);
    yk = Unicycle.x(2,k);
    thetak = Unicycle.x(3,k);
    % Next step
    if(k<500)
    xk_1 = xk + cos(thetak)*Unicycle.v(k)*Dt;
    yk_1 = yk + sin(thetak)*Unicycle.v(k)*Dt;
    thetak_1 = thetak + Unicycle.omega(k)*Dt;
    else
    xk_1 = xk + cos(thetak)*Unicycle.v(k)*Dt;
    yk_1 = yk + sin(thetak)*Unicycle.v(k)*Dt;
    thetak_1 = thetak + 0;%Unicycle.omega(k)*Dt;
    end
    % Storing
    Unicycle.x(1:3,k+1) = [xk_1; yk_1; thetak_1]; 
   
    xk = Unicycle.x(4,k);
    yk = Unicycle.x(5,k);
    thetak = Unicycle.x(6,k);
    % Next step
    if(k<500)
    xk_1 = xk + cos(thetak)*Unicycle.v(k)*Dt;
    yk_1 = yk + sin(thetak)*Unicycle.v(k)*Dt;
    thetak_1 = thetak + Unicycle.omega(k)*Dt;
    else
    xk_1 = xk + cos(thetak)*Unicycle.v(k)*Dt;
    yk_1 = yk + sin(thetak)*Unicycle.v(k)*Dt;
    thetak_1 = thetak + 0;%Unicycle.omega(k)*Dt;
    end
    % Storing
    Unicycle.x(4:6,k+1) = [xk_1; yk_1; thetak_1];
    
    %%%% MODELS UPDATE END
end

