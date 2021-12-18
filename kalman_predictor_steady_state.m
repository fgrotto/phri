function [x_kalman_predictor_ss]=kalman_predictor_steady_state(x_k_kminus1,P,y,A,C,Q,R)
    [P] = idare(A',C',Q,R,[], eye(3,3));
    K = A*P * C' * inv(C * P * C' + R);
           
    for i=2:size(y,1)
       x_kplus1_k = A*x_k_kminus1 + K *(y(i) - C * x_k_kminus1);
       x_kalman_predictor_ss(:,i)=x_kplus1_k;
       x_k_kminus1=x_kplus1_k;
    end
end