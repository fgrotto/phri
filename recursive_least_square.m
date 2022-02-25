function [beta] = recursive_least_square(Y,X, lambda)
    % Initial conditions
    beta_0 = [1; 1];
    P_0 = eye(2);
    N = length(Y);

    % First iterations
    e(1) = Y(1) - X(1,:)*beta_0;
    P{1} = (1/lambda)*(P_0 - (P_0*X(1,:)'*X(1,:)*P_0) / (lambda+X(1,:)*P_0*X(1,:)'));
    K{1} = P{1}*X(1,:)';
    beta{1} = beta_0+K{1}*e(1);

    % Recursive computation
    for k = 2:N
        e(k) = Y(k) - X(k,:)*beta{k-1};
        P{k} = (1/lambda)*(P{k-1} - (P{k-1}*X(k,:)'*X(k,:)*P{k-1}) / (lambda+X(k,:)*P{k-1}*X(k,:)'));
        K{k} = P{k}*X(k,:)';
        beta{k} = beta{k-1}+K{k}*e(k);
    end
end