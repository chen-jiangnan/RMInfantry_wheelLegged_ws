function p = params_lqr()
    p.Q = diag([1 1 500 100 5000 1]);
    p.R = diag([1 0.25]);
end