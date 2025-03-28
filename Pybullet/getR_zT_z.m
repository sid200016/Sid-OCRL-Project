function [R, T] = getR_zT_z(q, p)
    R = [cos(q) -sin(q) 0;sin(q) cos(q) 0; 0 0 1];
    T = [R p;[0 0 0 1]];

end