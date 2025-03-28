function x_new = rk4_dyn(x, tau, dt)
k1 = dt*forward_dynamics(x, tau);
k2 = dt*forward_dynamics(x + k1/2, tau);
k3 = dt*forward_dynamics(x + k2/2, tau);
k4 = dt*forward_dynamics(x+k3, tau);

x_new = x + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
t_new = wraptopi(x_new(1:6));
x_new(1:6) = t_new;
end