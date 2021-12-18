function [x_kalman_filter_ss]=kalman_filter_steady_state(x_k_k,P,y,A,C,Q,R)
    % Compute P infinity
    [P] = idare(A',C',Q,R,[], eye(3,3));
    
    % Kalman gain computation
    K = P * C' * inv(C * P * C' + R);
    
    for i=2:size(y,1)
       x_kplus1_kplus1 = A*x_k_k + K *(y(i) - C * A * x_k_k);
       x_kalman_filter_ss(:,i)=x_kplus1_kplus1;
       x_k_k=x_kplus1_kplus1;
    end
end