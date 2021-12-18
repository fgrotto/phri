function [x_kalman_smoother] = kalman_smoother(x_initial, P_initial, y, A, C, Q, R)
    % Set initial conditions for the smoother
    x_kplus1_kplus1{1} = x_initial;
    x_kplus1_k{1} = x_initial;
    P_kplus1_k{1} = P_initial;
    P_kplus1_kplus1{1} = P_initial;
    
    % Forward interation 0->N (kalman filter)
    for k = 2:size(y, 1)
        % Prediction step (a priori estimation)
        x_kplus1_k{k} = A * x_kplus1_kplus1{k - 1};
        P_kplus1_k{k} = A * P_kplus1_kplus1{k - 1} * A' + Q;
        
        % Kalman gain
        K{k} = P_kplus1_k{k} * C' * inv(C * P_kplus1_k{k} * C' + R);
        
        % Estimation step
        x_kplus1_kplus1{k} = x_kplus1_k{k} + K{k} * (y(k) - C * x_kplus1_k{k});
        P_kplus1_kplus1{k} = P_kplus1_k{k} - P_kplus1_k{k} * C' * inv(C * P_kplus1_k{k} * C' + R) * C * P_kplus1_k{k};
    end
    
    % Save the filter result in s
    xs{size(y, 1)} = x_kplus1_kplus1{size(y, 1)};
    Ps{size(y, 1)} = P_kplus1_kplus1{size(y, 1)};
    
    % Backward iteration from N -> 0
    for k = size(y, 1)-1:-1:1
        % Compute K bar 
        K_bar{k} = P_kplus1_kplus1{k} * A' * inv(P_kplus1_k{k + 1});
        
        % Compute the smoother
        xs{k} = x_kplus1_kplus1{k} + K_bar{k} * (xs{k + 1} - x_kplus1_k{k + 1});
        Ps{k} = P_kplus1_kplus1{k} + K_bar{k} * (Ps{k + 1} - P_kplus1_k{k + 1});
    end
    
    % Save the result in the correct format
    for i = 1:length(xs)
        x_kalman_smoother(:, i) = xs{i};
    end
end
