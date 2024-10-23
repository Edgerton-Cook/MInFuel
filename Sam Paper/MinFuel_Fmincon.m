%% Non-Convex Minfuel Problem
% Edgerton M. Cook
% Oct. 10 2024
close all; clear all; clc;
set(0,'DefaultFigureWindowStyle','docked');
color = {'[0, 0.4470, 0.7410]', '[0.8500, 0.3250, 0.0980]',...
    '[0.9290, 0.6940, 0.1250]', '[0.4940, 0.1840, 0.5560]',...
    '[0.4660, 0.6740, 0.1880]', '[0.3010, 0.7450, 0.9330]',...
    '[0.6350, 0.0780, 0.1840]'};

%% Parameters:
T = 4.5; 
u_max = 200; 
u_min = 50;
theta_max = 30; 
phi_max = 60;
m = 10;
N = 10;
r_0 = [25; 25; 30];
v_0 = [0; 0; -10];
r_f = [0; 0; 0];
v_f = [0; 0; 0];
g = [0; 0; 0; 0; 0; -9.81];

%% Constraint Definition
max_fuel = true;
pointing_angle = true;
glide_slope = true;

%% State-Space Matricies
dT = T/(N-1);
d = dT*eye(3);
d2 = ((d)^2)/2;
dm = d/m;

Ap_cont = [zeros(3,3), eye(3); zeros(3,6)];
Bp_cont = [zeros(3,3); eye(3)/m];
Cp_cont = zeros(6,6);
Dp_cont = zeros(6,3);

A_cont = [Ap_cont, g; zeros(1,7)];
B_cont = [Bp_cont; zeros(1,3)];
C_cont = zeros(7,7);
D_cont = zeros(7,3);

sys_cont = ss(A_cont, B_cont, C_cont, D_cont);
sys_dis = c2d(sys_cont, dT);

A = sys_dis.A;
B = sys_dis.B;

%% Convex Subproblem
[x_0, u_0] = MinFuel_con(N, r_0, r_f, v_0, v_f, A, B, u_max, theta_max, phi_max, max_fuel, glide_slope, pointing_angle);



