function t_new = wraptopi(theta)
t_new = zeros(length(theta), 1);
for i = 1:length(theta)
    t = theta(i);
    t_new(i) = mod(t + pi, 2*pi) - 2*pi;
end
end