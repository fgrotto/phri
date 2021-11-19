clear all;
close all;

data=load('master_slave_1kHz.txt');
pos=data(400:end, 2); % second column is the master position
vel=data(400:end, 3); % third column is the master velocity
Ts=0.001;

% Calculate euler approximations of velocities and accelerations
vel_euler = (pos(2:end)-pos(1:end-1))./Ts;
acc_euler = (vel(2:end)-vel(1:end-1))./Ts;

A=[1 Ts Ts^2/2; 0 1 Ts; 0 0 1];
C=[1 0 0];
q=10000;
Q=q*[Ts^3/6; Ts^2/2; Ts]*[Ts^3/6; Ts^2/2; Ts]';
R=1;

%% Kalman filter and predictor
x_kalman_filter=zeros(3,size(pos,1));
x_filter_old=[pos(1);0; 0];
P_filter_old=0.1*eye(3);

x_kalman_predictor=zeros(3,size(pos,1));
x_predictor_old=[pos(1); 0; 0];
P_predictor_old=0.1*eye(3);


for i=2:size(pos,1)
   [x_filter_new, P_filter_new]=kalman_filter(x_filter_old, P_filter_old, pos(i), A, C, Q, R); 
   x_kalman_filter(:,i)=x_filter_new;
   x_filter_old=x_filter_new;
   P_filter_old=P_filter_new;
   
   [x_predictor_new, P_predictor_new]=kalman_predictor(x_predictor_old, P_predictor_old, pos(i), A, C, Q, R);
   x_kalman_predictor(:,i)=x_predictor_new;
   x_predictor_old=x_predictor_new;
   P_predictor_old=P_predictor_new;
end

figure;
plot(vel);
hold on; plot(vel_euler); % very noisy
hold on; plot(x_kalman_filter(2,:),'.b');
hold on; plot(x_kalman_predictor(2,:),'.g');
legend('vel real','vel euler', 'vel kalman filter', 'vel kalman predictor')

figure;
hold on; plot(acc_euler); % very noisy
hold on; plot(x_kalman_filter(3,:),'.b');
hold on; plot(x_kalman_predictor(3,:),'.g');
legend('acc euler', 'acc kalman filter', 'acc kalman predictor')

%% Kalman filter steady state and predictor steady state

x_kalman_filter_ss=zeros(3,size(pos,1));
x_filter_old=[pos(1);0; 0];
P_filter_old=0.1*eye(3);

x_kalman_predictor_ss=zeros(3,size(pos,1));
x_predictor_old=[pos(1); 0; 0];
P_predictor_old=0.1*eye(3);

P_filter = A*P_filter_old*A'-A*P_filter_old*C'*inv(C*P_filter_old*C'+R)*C*P_filter_old*A'+Q;
P_predictor = A*P_predictor_old*A'-A*P_predictor_old*C'*inv(C*P_predictor_old*C'+R)*C*P_predictor_old*A'+Q;

for i=2:size(pos,1)
   [x_filter_new]=kalman_filter_steady_state(x_filter_old, P_filter, pos(i), A, C, R); 
   x_kalman_filter_ss(:,i)=x_filter_new;
   x_filter_old=x_filter_new;
   
   [x_predictor_new]=kalman_predictor_steady_state(x_predictor_old, P_predictor, pos(i), A, C, R);
   x_kalman_predictor_ss(:,i)=x_predictor_new;
   x_predictor_old=x_predictor_new;
end

figure;
plot(vel);
hold on; plot(vel_euler); % very noisy
hold on; plot(x_kalman_filter_ss(2,:),'.b');
hold on; plot(x_kalman_predictor_ss(2,:),'.g');
legend('vel real','vel euler', 'vel kalman filter ss', 'vel kalman predictor ss')

figure;
hold on; plot(acc_euler); % very noisy
hold on; plot(x_kalman_filter_ss(3,:),'.b');
hold on; plot(x_kalman_predictor_ss(3,:),'.g');
legend('acc euler', 'acc kalman filter ss', 'acc kalman predictor ss')

%% Kalman smoother

n = size(A,1);
N = size(pos,1);

xf{1} = zeros(n,1);
xp{1} = zeros(n,1);
Pp{1} = 0.01*eye(n,n);
Pf{1} = 0.01*eye(n,n);

for k=2:size(pos,1)-1
    xp{k} = A*xf{k-1};
    Pp{k} = ( A*Pf{k-1}*A' + Q +  (A*Pf{k-1}*A' + Q)')/2;
    
    K{k} = Pp{k}*C'*inv(C*Pp{k}*C' + R);
    xf{k} = xp{k} + K{k}*(pos(k) - C*xp{k});
    
    Pf{k} = Pp{k} - Pp{k}*C'*inv(C*Pp{k}*C' + R)*C*Pp{k};
    Pf{k} = (Pf{k} + Pf{k}')/2;
end

xs{N-1} = xf{N-1};
Ps{N-1} = Pf{N-1};

for k=size(pos,1)-2:-1:1
    barK{k} = Pf{k}*A'*inv(Pp{k+1});
    xs{k} = xf{k} + barK{k}*( xs{k+1} - xp{k+1} );
    Ps{k} = Pf{k} + barK{k}*( Ps{k+1} - Pp{k+1} );
end

x_kalman_smoother = zeros(n,length(xs));
for i=1:length(xs)
    x_kalman_smoother(:,i) = xs{i};
end

figure;
hold on; plot(vel_euler); % very noisy
hold on; plot(x_kalman_smoother(2,:),'.r');
legend('vel euler', 'vel kalman smoother')

figure;
hold on; plot(acc_euler); % very noisy
hold on; plot(x_kalman_smoother(3,:),'.r');
legend('acc euler', 'acc kalman smoother')