function [R, T] = getR_yT_y(q, p)
    R = [cos(q) 0 sin(q);0 1 0;-sin(q) 0 cos(q);];
    T = [R p;[0 0 0 1]];

end