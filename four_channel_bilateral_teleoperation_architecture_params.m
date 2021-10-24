% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Fip = 1;
% Sin frequency
Fc = 1; 

% Master controller
Bm = 0.2;
Km = 30;

% Human intention controller
Ph = 5;
Dh = 1;

% Slave controller
Bs = 0.2;
Ks = 30;

% Intertia of robot dynamics
Mm = 0.5;
Ms = 2;

% Make the real pole with re<0 to change the inertia of the robot
% dynamics verify this value by adding them to the simulink model
% Dm = 5;
% Ds = 10;
Dm = 0;
Ds = 0;

% Human impedance parameters
Jh = 0.5;
Bh = 70;
Kh = 2000;

% Environment impedance parameters
Je = 0;
Be = 100;
Ke = 200;

% High frequency pole
tau = 10000;