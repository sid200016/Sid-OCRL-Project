function C = coriolis(M, q, qdot)
l = length(M);
C = sym(zeros(size(M)));
for i = 1:l
    for j = 1:l
        for k = 1:l
            C(i, j) = C(i, j) + 0.5*(diff(M(i, j), q(k)) + diff(M(i, k), q(j)) ...
                - diff(M(k, j), q(i)))*qdot(k);

        end
    end

end