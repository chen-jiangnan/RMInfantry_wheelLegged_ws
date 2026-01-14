clc; clear;

p = params_model  

eqs = newton_euler();

%%acc equ
eqs = solve_acc(eqs);
eqs.dds_w;
eqs.ddtheta_l;
eqs.ddtheta_b;

%%liner state-space
mode = linerize_model(eqs,p);
A = mode.A
B = mode.B

p = params_lqr;
lqr_k = lqr(mode, p.Q, p.R)
