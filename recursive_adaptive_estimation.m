function [beta_adaptive] = recursive_adaptive_estimation(Y,X,Ts)
    g = 0.54; % g > 0
    beta_adaptive_0 = [1;1];

    e{1} = Y(1) - X(1,:)*beta_adaptive_0;
    beta_adaptive{1} = beta_adaptive_0 + Ts*g*X(1,:)'*e{1};

    for k = 2:length(Y)
        e{k} = Y(k) - X(k,:)*beta_adaptive{k-1};
        beta_adaptive{k} = beta_adaptive{k-1} + Ts*g*X(k,:)'*e{k};
    end
end