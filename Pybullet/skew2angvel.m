function w = skew2angvel(w_hat)
w = [w_hat(3, 2);w_hat(1, 3); w_hat(2, 1)];
end