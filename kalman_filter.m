function [x_kplus1_kplus1, P_kplus1_kplus1]=kalman_filter(x_k_k,P_k_k,y,A,C,Q,R)
    % Prediction step (A priori estimation
    x_kplus1_k = A*x_k_k;
    P_kplus1_k = A*P_k_k*A'+Q;

    % Compute the kalman gain
    K = P_kplus1_k * C' * inv(C * P_kplus1_k * C' + R);

    % Calculate the estimation step
    x_kplus1_kplus1 = x_kplus1_k + K *(y - C * x_kplus1_k);
    P_kplus1_kplus1 = P_kplus1_k - P_kplus1_k * C' * inv(C * P_kplus1_k * C' + R) * C * P_kplus1_k;
end