function [c,ceq] = MinFuel_nonlin(z, params)


u_max = params.u_max;
u_min = params.u_min;
theta_max = params.theta_max;
phi_max = params.phi_max;
N = params.N;
r_f = params.r_f;
n_state = params.n_state;
n_thr = params.n_thr;

n_start = n_state + 2;
n_tot = n_state + n_thr + 1;

x = [];
u = [];

for q = 1:N
    p = 10*(q-1);
    x = [x,z(p+1:p+n_state,1)];
end

for q = 1:N-1
    p = 10*(q-1);
    u = [u,z(p+n_start:p+n_tot,1)];
end

c = zeros(4*(N-1),1);
for q = 1:N-1
    p = 4*(q-1);
    c(p+1) = norm(u(:,q)) - u_max;
    c(p+2) = u_min - norm(u(:,q));
    c(p+3) = cosd(theta_max) * norm(u(:,q)) - (u(:,q)'*[0; 0; 1]);
    c(p+4) = cosd(phi_max) * norm(x(1:3,q)-r_f) - ((x(1:3,q)-r_f)'*[0; 0; 1]);
end
ceq = [];