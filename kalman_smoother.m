function [x_kalman_smoother] = kalman_smoother(x_initial, P_initial, y, A, C, Q, R)
    x_kplus1_kplus1{1} = x_initial;
    x_kplus1_k{1} = x_initial;
    P_kplus1_k{1} = P_initial;
    P_kplus1_kplus1{1} = P_initial;

    for k = 2:size(y, 1) - 1
        x_kplus1_k{k} = A * x_kplus1_kplus1{k - 1};
        P_kplus1_k{k} = A * P_kplus1_kplus1{k - 1} * A' + Q;

        K{k} = P_kplus1_k{k} * C' * inv(C * P_kplus1_k{k} * C' + R);

        x_kplus1_kplus1{k} = x_kplus1_k{k} + K{k} * (y(k) - C * x_kplus1_k{k});
        P_kplus1_kplus1{k} = P_kplus1_k{k} - P_kplus1_k{k} * C' * inv(C * P_kplus1_k{k} * C' + R) * C * P_kplus1_k{k};
    end

    xs{size(y, 1) - 1} = x_kplus1_kplus1{size(y, 1) - 1};
    Ps{size(y, 1) - 1} = P_kplus1_kplus1{size(y, 1) - 1};

    for k = size(y, 1) - 2:-1:1
        K_bar{k} = P_kplus1_kplus1{k} * A' * inv(P_kplus1_k{k + 1});
        xs{k} = x_kplus1_kplus1{k} + K_bar{k} * (xs{k + 1} - x_kplus1_k{k + 1});
        Ps{k} = P_kplus1_kplus1{k} + K_bar{k} * (Ps{k + 1} - P_kplus1_k{k + 1});
    end

    for i = 1:length(xs)
        x_kalman_smoother(:, i) = xs{i};
    end

end
