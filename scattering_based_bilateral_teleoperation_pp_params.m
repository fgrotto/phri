% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
A = 1;

% Low pass frequency cuff off
Fip = 1;
% Sin frequency
Fc = 1; 

% Human intention controller (PI)
Ph = 2010;
Dh = 100;

% Master controller
Bm = 0.8;
Km = 1;

% Slave controllerxs
Bs = 0.8*4;
Ks = 4;

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
Be = 0; %100;
Ke = 50; %200;

% Characteristic impedance
b = 0.234;

% Sampling time
Ts = 0.001;

% Noise parameters
VarianceForces = 0.01;
VarianceVelocities = 0.000001;