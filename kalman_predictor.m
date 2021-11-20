function [x_kalman_predictor]=kalman_predictor(x_k_kminus1,P_k_kminus1,y,A,C,Q,R)
    for i=2:size(y,1)
        K = A*P_k_kminus1 * C' * inv(C * P_k_kminus1 * C' + R);

        x_kplus1_k = A*x_k_kminus1 + K*(y(i) - C * x_k_kminus1);
        P_kplus1_k = A*P_k_kminus1*A' - A*P_k_kminus1 * C' * inv(C * P_k_kminus1 * C' + R) * C * P_k_kminus1*A'+Q;

        x_kalman_predictor(:,i)=x_kplus1_k;
        x_k_kminus1=x_kplus1_k;
        P_k_kminus1=P_kplus1_k;
    end
end