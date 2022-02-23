function [x_kalman_filter]=kalman_filter(x_k_k,P_k_k,y,A,C,Q,R)
    for i=2:size(y,1)
        % Prediction step (A priori estimation)
        x_kplus1_k = A*x_k_k;
        P_kplus1_k = A*P_k_k*A'+Q;

        % Compute the kalman gain
        K = P_kplus1_k * C' * inv(C * P_kplus1_k * C' + R);

        % Calculate the estimation step
        x_kplus1_kplus1 = x_kplus1_k + K *(y(i) - C * x_kplus1_k);
        P_kplus1_kplus1 = P_kplus1_k - P_kplus1_k * C' * inv(C * P_kplus1_k * C' + R) * C * P_kplus1_k;
        
        % Handle next iteration
        x_kalman_filter(:,i)=x_kplus1_kplus1;
        x_k_k=x_kplus1_kplus1;
        P_k_k=P_kplus1_kplus1;
    end
end