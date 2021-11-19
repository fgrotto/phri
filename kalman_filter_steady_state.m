function [x_kplus1_kplus1]=kalman_filter_steady_state(x_k_k,P,y,A,C,R)
    K = P * C' * inv(C * P * C' + R);
    x_kplus1_kplus1 = A*x_k_k + K *(y - C * A * x_k_k);
end