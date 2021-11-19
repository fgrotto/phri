function [x_kplus1_k, P_kplus1_k]=kalman_predictor(x_k_kminus1,P_k_kminus1,y,A,C,Q,R)
    K = A*P_k_kminus1 * C' * inv(C * P_k_kminus1 * C' + R);
    
    x_kplus1_k = A*x_k_kminus1 + K *(y - C * x_k_kminus1);
    P_kplus1_k = A*P_k_kminus1*A' - A*P_k_kminus1 * C' * inv(C * P_k_kminus1 * C' + R) * C * P_k_kminus1*A'+Q;
end