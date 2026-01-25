function p = params_lqr()
    p.Q = diag([100 1 500 100 5000 1]);
    p.R = diag([240 25]);
end