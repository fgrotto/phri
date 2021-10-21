% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Master controller
Bm = 1;
Km = 1;

% Slave controller
Bs = 1;
Ks = 1;

% Intertia of robot dynamics
Mm = 0.5;
Ms = 2;

% Make the real pole with re<0 to change the inertia of the robot
% dynamics verify this value by adding them to the simulink model
Dm = 5;
Ds = 10;

% Human impedance parameters
Jh = 0.5;
Bh = 70;
Kh = 2000;

% Environment impedance parameters
Je = 0;
Be = 100;
Ke = 200;

