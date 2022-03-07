% Sample parameters for the scattering based bilateral teleoperation
clear all;
close all;

% Input function amplitude parameter (sin or step with low pass filter)
A = 1;
% Low pass frequency cuff off
Fip = 5;
% Sin frequency
Fc = 0.1; 
delay = 1;

% Cut off frequency for filter after each discrete derivative
Filter = 100; %Hz

% Human intention controller (PI)
Ph = 5;
Dh = 0;
Ih = 2;

% Master controller
Bm = 0; %0.8;
Km = 0; %1;

% Slave controller
Bs = 40;
Ks = 50;

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

% Human impedance parameters
Jh = 0;  %0.5;
Bh = 1.5;   %70;
Kh = 1;     %2000;

% Environment impedance parameters
Je = 0;
Be = 10; %100;
Ke = 200; %200;

alpha = 1;
beta = 0.2;
h_limit = 1;
H_master_init = 100;
H_slave_init = 100;

% Sampling time
Ts = 0.001;

VarianceForces = 0.01;
VarianceVelocities = 0.00001;