clear all;
close all;

data=load('master_slave_1kHz.txt');
pos=data(400:end, 2); % second column is the master position
vel=data(400:end, 3); % third column is the master velocity
Ts=0.001;

% Calculate euler approximations of velocities and accelerations
vel_euler = (pos(2:end)-pos(1:end-1))./Ts;
acc_euler = (vel(2:end)-vel(1:end-1))./Ts;
vel_euler = lowpass(vel_euler, 8, 1/Ts);
acc_euler = lowpass(acc_euler, 5, 1/Ts);

A=[1 Ts Ts^2/2; 0 1 Ts; 0 0 1];
C=[1 0 0];
q=1000;
Q=q*[Ts^3/6; Ts^2/2; Ts]*[Ts^3/6; Ts^2/2; Ts]';
R=1;

%% Kalman filter and predictor
x_filter_old=[pos(1);0; 0];
P_filter_old=0.1*eye(3);

x_predictor_old=[pos(1); 0; 0];
P_predictor_old=0.1*eye(3);

x_kalman_filter=kalman_filter(x_filter_old, P_filter_old, pos, A, C, Q, R);  
x_kalman_predictor=kalman_predictor(x_predictor_old, P_predictor_old, pos, A, C, Q, R);

% figure;
% subplot(1,2,1);
% plot(vel)
% hold on; plot(vel_euler); % very noisy
% hold on; plot(x_kalman_filter(2,:));
% hold on; plot(x_kalman_predictor(2,:));
% legend('vel encoder', 'vel euler', 'vel kalman filter', 'vel kalman predictor')
% 
% subplot(1,2,2);
% hold on; plot(acc_euler); % very noisy
% hold on; plot(x_kalman_filter(3,:));
% hold on; plot(x_kalman_predictor(3,:));
% legend('acc euler', 'acc kalman filter', 'acc kalman predictor')

%% Kalman filter steady state and predictor steady state

x_filter_old=[pos(1);0; 0];
P_filter_old=0.1*eye(3);

x_predictor_old=[pos(1); 0; 0];
P_predictor_old=0.1*eye(3);

x_kalman_filter_ss=kalman_filter_steady_state(x_filter_old, P_filter_old, pos, A, C, Q, R); 
x_kalman_predictor_ss=kalman_predictor_steady_state(x_predictor_old, P_predictor_old, pos, A, C, Q, R);

% figure;
% plot(vel);
% hold on; plot(vel_euler); % very noisy
% hold on; plot(x_kalman_filter_ss(2,:),'b');
% hold on; plot(x_kalman_predictor_ss(2,:),'g');
% legend('vel real','vel euler', 'vel kalman filter ss', 'vel kalman predictor ss')
% 
% figure;
% hold on; plot(acc_euler); % very noisy
% hold on; plot(x_kalman_filter_ss(3,:),'b');
% hold on; plot(x_kalman_predictor_ss(3,:),'g');
% legend('acc euler', 'acc kalman filter ss', 'acc kalman predictor ss')

% Compare kalman filters
time = 0:1:size(vel_euler);
time = time*Ts;
figure;
subplot(1,2,1);
plot(time, vel)
hold on; plot(time(1:end-1),vel_euler); % very noisy
hold on; plot(time,x_kalman_filter(2,:));
hold on; plot(time,x_kalman_filter_ss(2,:));
ylabel('velocity [rad/s]');
xlabel('time [s]');
legend('vel encoder', 'vel euler', 'vel kalman filter', 'vel kalman filter ss')

subplot(1,2,2);
hold on; plot(time(1:end-1), acc_euler); % very noisy
hold on; plot(time, x_kalman_filter(3,:));
hold on; plot(time, x_kalman_filter_ss(3,:));
ylabel('acceleration [rad/s^2]');
xlabel('time [s]');
legend('acc euler', 'acc kalman filter', 'acc kalman filter ss')

% Compare kalman predictors
figure;
subplot(1,2,1);
plot(time, vel)
hold on; plot(time(1:end-1),vel_euler); % very noisy
hold on; plot(time,x_kalman_predictor(2,:));
hold on; plot(time,x_kalman_predictor_ss(2,:));
ylabel('velocity [rad/s]');
xlabel('time [s]');
legend('vel encoder', 'vel euler', 'vel kalman predictor', 'vel kalman predictor ss')

subplot(1,2,2);
hold on; plot(time(1:end-1), acc_euler); % very noisy
hold on; plot(time, x_kalman_predictor(3,:));
hold on; plot(time, x_kalman_predictor_ss(3,:));
ylabel('acceleration [rad/s^2]');
xlabel('time [s]');
legend('acc euler', 'acc kalman predictor', 'acc kalman predictor ss')


%% Kalman smoother

x_kalman_smoother=kalman_smoother(x_predictor_old, P_predictor_old, pos, A, C, Q, R);

figure;
subplot(1,2,1);
hold on; plot(time(1:end-1),vel_euler); % very noisy
hold on; plot(time,x_kalman_filter(2,:));
hold on; plot(time,x_kalman_smoother(2,:));
ylabel('velocity [rad/s]');
xlabel('time [s]');
legend('vel euler', 'vel kalman filter', 'vel kalman smoother')

subplot(1,2,2);
hold on; plot(time(1:end-1), acc_euler); % very noisy
hold on; plot(time, x_kalman_filter(3,:));
hold on; plot(time, x_kalman_smoother(3,:));
ylabel('acceleration [rad/s^2]');
xlabel('time [s]');
legend('acc euler', 'acc kalman filter', 'acc kalman smoother')


%% All in one

% figure;
% hold on; plot(x_kalman_filter(2,:));
% % hold on; plot(x_kalman_filter_ss(2,:),'b');
% hold on; plot(x_kalman_predictor(2,:));
% hold on; plot(x_kalman_smoother(2,:));
% legend('vel kalman filter', 'vel kalman predictor', 'vel kalman smoother')
% 
% figure;
% hold on; plot(x_kalman_filter(3,:));
% % hold on; plot(x_kalman_filter_ss(3,:),'b');
% hold on; plot(x_kalman_predictor(3,:));
% hold on; plot(x_kalman_smoother(3,:));
% legend('acc kalman filter', 'acc kalman predictor', 'acc kalman smoother')

%% Construction of simple low pass filter (discrete domain)
% s = tf('s');
% Fc = 10;
% G = 2*pi*Fc/(s+2*pi*Fc);
% 
% p_c = -2*pi*Fc;    % pole in continuous domain
% p_z = exp(p_c*Ts); % pole in discrete domain
% 
% figure;
% bode(G);
% hold on;
% P = c2d(G,Ts); % Move continuous to discrete
% bode(P);
