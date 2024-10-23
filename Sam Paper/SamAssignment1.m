%% SCP Assignment 1
% Edgerton M. Cook
% May 12th, 2024

clc; clear all; close all;


set(0, 'DefaultFigureWindowStyle', 'docked');

color = {'[0, 0.4470, 0.7410]', '[0.8500, 0.3250, 0.0980]',...
    '[0.9290, 0.6940, 0.1250]', '[0.4940, 0.1840, 0.5560]',...
    '[0.4660, 0.6740, 0.1880]', '[0.3010, 0.7450, 0.9330]',...
    '[0.6350, 0.0780, 0.1840]'};


%% Parameters
T = 4.5; 
u_max = 200; 
% u_min = 50;
theta_max = 30; 
phi_max = 60;
m = 10;
N = 10;
r_0 = [25; 25; 30];
v_0 = [0; 0; -10];
r_f = [0; 0; 0];
v_f = [0; 0; 0];
g = [0; 0; 0; 0; 0; -9.81];

%% Constraints
max_fuel = true;
pointing_angle = true;
glide_slope = true;

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

%% Solver
[x, u] = MinFuel_con(N, r_0, r_f, v_0, v_f, A, B, u_max, theta_max, phi_max, max_fuel, glide_slope, pointing_angle);

%% Plotting
pos_x = x(1,:);
pos_y = x(2,:);
pos_z = x(3,:);

x_rate = x(4,:);
y_rate = x(5,:);
z_rate = x(6,:);

t = 0:dT:T;

thr_x = [u(1,:)];
thr_y = [u(2,:)];
thr_z = [u(3,:)];


% Change from a for loop to calling dimension of norm - Improve efficiency
for i = 1:length(t)
u_mag(i) = norm([thr_x(i); thr_y(i); thr_z(i)]);
end

if pointing_angle
    [X_point, Y_point, Z_point] = plotcones(theta_max, pos_x, pos_y, pos_z, 1);
end

if glide_slope
    [X_glide, Y_glide, Z_glide] = plotcones(phi_max, 0, 0, 0, max(pos_z));
end

figure(1)
tiledlayout(3,3)
nexttile([2,3])
plot3(pos_x,pos_y,pos_z,':o','MarkerSize',2);
hold on
plot3(r_0(1),r_0(2),r_0(3),'o',"MarkerSize",10,'Color',color{3});
plot3(r_f(1),r_f(2),r_f(3),'o',"MarkerSize",10,'Color',color{3});
quiver3(pos_x,pos_y,pos_z,thr_x,thr_y,thr_z,'LineWidth',1.25,'ShowArrowHead','off','Color',color{2})
if pointing_angle
    for i = 1:length(pos_x)
        surf(X_point{i},Y_point{i},Z_point{i})
        alpha(0.5)
        colormap('summer')
    end
end
if glide_slope
    surf(X_glide{1},Y_glide{1},Z_glide{1})
    alpha(0.15)
    colormap('summer')
end
title('Drone Trajectory','FontSize',12)
set(gca,'FontSize',12)
nexttile
plot(t,pos_x)
title('X','FontSize',12)
set(gca,'FontSize',12)
ylabel('x-pos (m)','FontSize',12)
xlabel('time (s)','FontSize',12)
nexttile
plot(t,pos_y)
title('Y','FontSize',12)
set(gca,'FontSize',12)
ylabel('y-pos (m)','FontSize',12)
xlabel('time (s)','FontSize',12)
nexttile
plot(t,pos_z)
title('Z','FontSize',12)
set(gca,'FontSize',12)
ylabel('z-pos (m)','FontSize',12)
xlabel('time (s)','FontSize',12)

figure(2)
plot(t(1:end-1),u_mag(1:end-1))
title('Magnitude of Thrust vs Time')
ylabel('mag(u)')
xlabel('Time (s)')

% figure(3)
% f = tiledlayout(3,2);
% 
% nexttile
% plot(t,x_rate)
% title('$\dot{x}$','Interpreter','latex')
% set(gca,'FontSize',12)
% ylabel('x-rate (m/s)')
% xlabel('time (s)')
% 
% nexttile
% plot(t,thr_x)
% title('X Force')
% set(gca,'FontSize',12)
% ylabel('x-force ( )')
% xlabel('time (s)')
% 
% nexttile
% plot(t,y_rate)
% title('$\dot{y}$','Interpreter','latex')
% set(gca,'FontSize',12)
% ylabel('y-rate (m/s)')
% xlabel('time (s)')
% 
% nexttile
% plot(t,thr_y)
% title('Y Force')
% set(gca,'FontSize',12)
% ylabel('y-force ( )')
% xlabel('time (s)')
% 
% nexttile
% plot(t,z_rate)
% title('$\dot{z}$','Interpreter','latex')
% set(gca,'FontSize',12)
% ylabel('z-rate (m/s)')
% xlabel('time (s)')
% 
% nexttile
% plot(t,thr_z)
% title('Z Force')
% set(gca,'FontSize',12)
% ylabel('z-force ( )')
% xlabel('time (s)')
% 
% 
% 
% 
% 
