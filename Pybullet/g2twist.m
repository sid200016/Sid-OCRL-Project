function V = g2twist(Vhat)
        
        w_hat = Vhat(1:3, 1:3);
        w = skew2angvel(w_hat);
        v = Vhat(1:3, 4);
        V = [v;w];
end