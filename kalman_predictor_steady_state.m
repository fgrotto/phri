function [x_kplus1_k]=kalman_predictor_steady_state(x_k_kminus1,P,y,A,C,R)
    K = A*P * C' * inv(C * P * C' + R);
    x_kplus1_k = A*x_k_kminus1 + K *(y - C * x_k_kminus1);
end