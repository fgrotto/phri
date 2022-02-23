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
Ph = 50500; %20000;
Dh = 500;

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

% Inner force loop
Cmf = 0; %1;
Csf = 0; %0.2;

Ts = 0.001;

VarianceForces = 0.01;
VarianceVelocities = 0.000001;

% s = tf('s');
% Zm = Mm*s+Dm;
% Zs = Ms*s+Ds;
% Cm = (Bm*s+Km)/s;
% Cs = (Bs*s+Ks)/s;
% C4 = -(Mm*s^2+(Bm+Dm)*s+Km)/s;
% C2 = 1;
% C1 = (Ms*s^2+(Bs+Ds)*s+Ks)/s;
% C3 = 1;
% D = 1/(C1+C3*Zm+C3*Cm);
% 
% H11 = (Zm+Cm)*D*(Zs+Cs-C3*C4)+C4
% H12 = -(Zm+Cm)*D*(1-C3*C2)-C2
% H21 = minreal(D*(Zs+Cs-C3*C4))
% H22 = -D*(1-C3*C2)
% 
% Zwidth = (H12*H21 - H11*H22) / (H22*H21)

% G = w_n^2/(s^2+2*w_n*xi*s+w_n^2)