function [x_kalman_filter_ss]=kalman_filter_steady_state(x_k_k,P,y,A,C,Q,R)
    P = A*P*A'-A*P*C'*inv(C*P*C'+R)*C*P*A'+Q;

    for i=2:size(y,1)
       K = P * C' * inv(C * P * C' + R);
       x_kplus1_kplus1 = A*x_k_k + K *(y(i) - C * A * x_k_k);
       x_kalman_filter_ss(:,i)=x_kplus1_kplus1;
       x_k_k=x_kplus1_kplus1;
    end
end