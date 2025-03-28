function xdot = forward_dynamics(x, tau)
q = x(1:6);
dq = x(7:12);
M = compute_M(x);
C = compute_C(x);
N = compute_N(x);
ddq = M\(tau - C*dq-N);
xdot = [dq;ddq];
end