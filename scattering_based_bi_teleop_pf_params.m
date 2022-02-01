% Sample parameters for the scattering based bilateral teleoperation
clear all;
close all;

% Input function amplitude parameter (sin or step with low pass filter)
A = 1;
% Low pass frequency cuff off
Fip = 1;
% Sin frequency
Fc = 1; 
delay = 1;

% Cut off frequency for filter after each discrete derivative
Filter = 5; %Hz

% Human intention controller (PI)
Ph = 23;
Dh = 17;

% Slave controller
Bs = 3;
Ks = 110;

% Inertia of robot dynamics
Mm = 0.5;
Ms = 2;

% Make the real pole with re<0 to change the inertia of the robot
% dynamics verify this value by adding them to the simulink model
% Dm = 5;
% Ds = 10;
Dm = 0;
Ds = 0;

% Human impedance parameters
% Jh = 0.05;
% Bh = 1.5;
% Kh = 0;

% Original human impedance parameters
Jh = 0.05;  %0.5;
Bh = 1.5;   %70;
Kh = 0;     %2000;

% Environment impedance parameters
Je = 0;
Be = 10; %100;
Ke = 200; %200;

% Characteristic impedance
b = 1;

% Sampling time
Ts = 0.001;

%% Slave tuning
s = tf('s');
P = 1/(Ms*s);
C = Ks+Bs/s;

T = (C*P)/(1+C*P);
step(T)