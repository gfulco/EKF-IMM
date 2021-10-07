function [IMM] = instance_IMM(Unicycle,num,M, steps)

    IMM.N = M; %number of filters.
    IMM.PI = randfixedsum(n,m,s,a,b); % probability Matrix
    IMM.mu_hat = zeros(IMM.N,steps);
    IMM.mu_hat(:,1) = [0.8 0.2]';
    IMM.c = zeros(IMM.N,steps);
    IMM.mu_tilde = zeros(IMM.N);
    IMM.P_hat = zeros(9,9);
    IMM.x_hat = zeros(3*num,steps);
    IMM.c_bar = zeros(IMM.N,steps);
    
    x_hat_0a = zeros(length(IMM.x_hat)/num, steps);
    x_hat_0b = zeros(length(IMM.x_hat)/num, steps);
    
    IMM.x_hat(:,1) = Unicyclea.x_hat(:,1)*IMM.mu_hat(1,1)+Unicycleb.x_hat(:,1)*IMM.mu_hat(2,1);
    IMM.P_hat = IMM.mu_hat(1,1)*(Unicyclea.P_hat(:,:,1)+(Unicyclea.x_hat(:,1)-IMM.x_hat(:,1))*(Unicyclea.x_hat(:,1)-IMM.x_hat(:,1))')+...
                IMM.mu_hat(2,1)*(Unicycleb.P_hat(:,:,1)+(Unicycleb.x_hat(:,1)-IMM.x_hat(:,1))*(Unicycleb.x_hat(:,1)-IMM.x_hat(:,1))');
            




end