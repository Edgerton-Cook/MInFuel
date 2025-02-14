function [x, u] = MinFuel_con(params, constraints)
    

N = params.N;
r_0 = params.r_0;
r_f = params.r_f;
v_0 = params.v_0;
v_f = params.v_f;
A = params.A;
B = params.B;
u_max = params.u_max;
theta_max = params.theta_max;
phi_max = params.phi_max;
max_fuel = constraints.max_fuel;
glide_slope = constraints.glide_slope;
pointing_angle = constraints.pointing_angle;


cvx_begin
    variables x(6,N) u(3,N-1)
    minimize( sum(norm(u)) )
    subject to 
        for n = 2:N
            [x(:,n);1] == A*[x(:,n-1);1] + B*u(:,n-1);
        end
            x(:,1) == [r_0; v_0];
            x(:,N) == [r_f; v_f];
        for n = 1:N-1
            if max_fuel
                norm(u(:,n)) <= u_max
            end
            if pointing_angle
                (u(:,n)'*[0; 0; 1]) >= cosd(theta_max) * norm(u(:,n));
            end
            if glide_slope
                ((x(1:3,n)-r_f)'*[0; 0; 1]) >= cosd(phi_max) * norm(x(1:3,n)-r_f);
            end
        end
cvx_end

end