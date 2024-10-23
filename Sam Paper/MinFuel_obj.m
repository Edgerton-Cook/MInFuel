function [fuel_cons] = MinFuel_obj(z, params)


N = params.N;
n_state = params.n_state; 
n_thr = params.n_thr;

n_start = n_state + 2;
n_tot = n_state + n_thr + 1;

u = [];

for q = 1:N-1
    p = 10*(q-1);
    u = [u,z(p+n_start:p+n_tot,1)];
end

fuel_cons = sum(norm(u));

end