clear all;
close all;

data = load('master_slave_1kHz.txt');
pos = data(400:end, 2); % second column is the master position
v = data(400:end, 4); % second column is the master voltage
Ts = 0.001;

A = [1 Ts Ts^2/2; 0 1 Ts; 0 0 1];
C = [1 0 0];
q = 15000;
Q = q * [Ts^3/6; Ts^2/2; Ts] * [Ts^3/6; Ts^2/2; Ts]';
R = 1;

% Kalman filter to estimate w and dot_w from the position measurements
x_filter_old = [pos(1); 0; 0];
P_filter_old = 0.1 * eye(3);

x_predictor_old = [pos(1); 0; 0];
P_predictor_old = 0.1 * eye(3);

x_kalman_filter = kalman_filter(x_filter_old, P_filter_old, pos, A, C, Q, R);

% get w and dot_w from the kalman filter
w = x_kalman_filter(2, :);
dot_w = x_kalman_filter(3, :);

%% Apply least square to get the DC motor parameters

% theta = [tau/k; 1/k]
X = [w' dot_w'];
Y = lowpass(v, 5, 1/Ts);

% compute the least square formulation
beta_hat = inv(X' * X) * X' * Y;

k_ls = 1 / beta_hat(2, 1);
tau_ls = k_ls * beta_hat(1, 1);

time = 0:1:length(Y);
time = time*Ts;
figure;
plot(time(1:end-1), Y);
hold on;
plot(time(1:end-1), X*beta_hat);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'ls');
fprintf("\nLeast square k = %f tau = %f\n", k_ls, tau_ls);

%% Apply the recursive least square to get the DC motor parameters

lambda = 1;
beta_rls = recursive_least_square(Y, X, lambda);
N = length(Y);

k_rls = 1 / beta_rls{N}(2);
tau_rls = k_ls * beta_rls{N}(1);

for i = 1:length(beta_rls)
    y_rls(i,:) = X(i,:)*beta_rls{i};
end

figure;
plot(time(1:end-1), Y);
hold on;
plot(time(1:end-1), y_rls);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'rls');
fprintf("\nRecursive least square k = %f tau = %f\n", k_rls, tau_rls);

%% Apply the discrete adaptive algorithm for the estimation

beta_adaptive = recursive_adaptive_estimation(Y, X, Ts);
N = length(Y);

k_ad = 1 / beta_adaptive{N}(2);
tau_ad = k_ad * beta_adaptive{N}(1);

for i = 1:length(beta_adaptive)
    y_ad(i,:) = X(i,:)*beta_adaptive{i};
end

figure;
plot(time(1:end-1),Y);
hold on;
plot(time(1:end-1),y_ad);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'adaptive');
fprintf("\nAdaptive estimation k = %f tau = %f\n", k_ad, tau_ad);

%% Compare Models

figure;
subplot(1,3,1);
plot(time(1:end-1), Y);
hold on;
plot(time(1:end-1), X*beta_hat);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'ls');

subplot(1,3,2);
plot(time(1:end-1), Y);
hold on;
plot(time(1:end-1), y_rls);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'rls');

subplot(1,3,3);
plot(time(1:end-1),Y);
hold on;
plot(time(1:end-1), y_ad);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'adaptive');

% Forgetting factor

lambda = 0.95;
beta_rls_f = recursive_least_square(Y, X, lambda);

for i = 1:length(beta_rls_f)
    y_rls_f(i,:) = X(i,:)*beta_rls_f{i};
end

figure;
plot(time(1:end-1), Y);
hold on;
plot(time(1:end-1), y_rls);
hold on;
plot(time(1:end-1), y_rls_f);
ylabel('y [model]');
xlabel('time [s]');
legend('model', 'rls lambda 1', 'rls lambda 0.9');


