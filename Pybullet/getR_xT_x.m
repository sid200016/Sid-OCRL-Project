function [R, T] = getR_xT_x(q, p)
    R = [1 0 0;0 cos(q) -sin(q) ; 0 sin(q) cos(q)];
    T = [R p;[0 0 0 1]];

end