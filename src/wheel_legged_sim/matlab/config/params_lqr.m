function p = params_lqr()
    p.Q = diag([1 1 1 800 5000 1]);
    p.R = diag([10 2.5]);
end