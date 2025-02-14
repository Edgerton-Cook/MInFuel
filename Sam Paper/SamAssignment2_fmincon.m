%% Non-Convex Minfuel Problem
% Edgerton M. Cook
% Oct. 10 2024
close all; clear all; clc;
% keyboard
set(0,'DefaultFigureWindowStyle','docked');
color = {'[0, 0.4470, 0.7410]', '[0.8500, 0.3250, 0.0980]',...
    '[0.9290, 0.6940, 0.1250]', '[0.4940, 0.1840, 0.5560]',...
    '[0.4660, 0.6740, 0.1880]', '[0.3010, 0.7450, 0.9330]',...
    '[0.6350, 0.0780, 0.1840]'};

%% Storing Parameters for Func
params.T = 4.5; 
params.u_max = 200; 
params.u_min = 100;
params.theta_max = 30; 
params.phi_max = 60;
params.m = 10;
params.N = 10;
params.r_0 = [25; 25; 30];
params.v_0 = [0; 0; -10];
params.r_f = [0; 0; 0];
params.v_f = [0; 0; 0];
params.g = [0; 0; 0; 0; 0; -9.81];
params.n_state = 6;
params.n_thr = 3;


%% Pulling Params for Main Script

N = params.N;
m = params.m;
g = params.g;
T = params.T;
r_0 = params.r_0;
r_f = params.r_f;
v_0 = params.v_0;
v_f = params.v_f;
n_state = params.n_state;
n_thr = params.n_thr;
n_start = n_state + 2;
n_tot = n_state + n_thr + 1;



%% Constraint Definition
constraints.max_fuel = true;
constraints.pointing_angle = true;
constraints.glide_slope = true;

%% State-Space Matricies
dT = T/(N-1);
d = dT*eye(3);
d2 = ((d)^2)/2;
dm = d/params.m;

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

params.A = sys_dis.A;
params.B = sys_dis.B;

% Pulling for main script
A = params.A;
B = params.B;

%% Convex Subproblem
[x_0, u_0] = MinFuel_con(params, constraints);

z_0 = [];

for q = 1:2*N-1
    if mod(q,2) ~= 0
        z_0 = [z_0;x_0(:,(q+1)/2);1];
    elseif mod(q,2) == 0
        z_0 = [z_0;u_0(:,q/2)];
    end
end

Aeq = cell(params.N-1,2*params.N-1);

for q = 1:params.N-1
    Aeq{q,2*q-1} = params.A;
    Aeq{q,2*q} = params.B;
    Aeq{q,2*q+1} = -eye(7,7);
end


A_0 = cell(1,length(Aeq));
A_f = cell(1,length(Aeq));
A_0{1,1} = eye(n_state+1,n_state+1);
A_f{end,end} = eye(n_state+1,n_state+1);

Aeq = [Aeq;A_0;A_f];

% Sets all empty entries of Aeq = 0
for m = 1:length(Aeq(:,1))
    for q = 1:length(Aeq(1,:))
        if isempty(Aeq{m,q})
            if mod(q,2) ~= 0
                Aeq{m,q} = zeros(n_state+1,n_state+1);
            elseif mod(q,2) == 0
                Aeq{m,q} = zeros(n_state+1,n_thr);
            end
        end
    end
end

Aeq = cell2mat(Aeq);

l_z = length(z_0);

b = zeros((n_state+1)*(N-1),1);
b_0 = [r_0;v_0;1];
b_f = [r_f;v_f;1];
beq = [b;b_0;b_f];

my_func = @(z) MinFuel_obj(z, params);
my_nonlin = @(z) MinFuel_nonlin(z, params);
[z_f] = fmincon(my_func,z_0,[],[],Aeq,beq,[],[],my_nonlin);

x_f = [];
u_f = [];

for q = 1:N
    p = 10*(q-1);
    x_f = [x_f,z_f(p+1:p+n_state,1)];
end

for q = 1:N-1
    p = 10*(q-1);
    u_f = [u_f,z_f(p+n_start:p+n_tot,1)];
end
