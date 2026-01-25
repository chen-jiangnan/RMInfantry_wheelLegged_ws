function sys = linerize_model(eqs_acc,params)

syms s_w ds_w dds_w 
syms theta_l dtheta_l ddtheta_l
syms theta_b dtheta_b ddtheta_b
syms Tw Tp

syms g Mw Ml Mb Iw Il Ib Lwc Lcp Lpb Rw

dds_w = eqs_acc.dds_w;
ddtheta_l = eqs_acc.ddtheta_l;
ddtheta_b = eqs_acc.ddtheta_b;

% A = subs(jacobian([ds_w,dds_w,dtheta_l,ddtheta_l,dtheta_b,ddtheta_b],[s_w,ds_w,theta_l,dtheta_l,theta_b,dtheta_b]),[s_w,ds_w,theta_l,dtheta_l,theta_b,dtheta_b,Tw,Tp],[0,0,0,0,0,0,0,0]);
% B = subs(jacobian([ds_w,dds_w,dtheta_l,ddtheta_l,dtheta_b,ddtheta_b],[Tw,Tp]),[s_w,ds_w,theta_l,dtheta_l,theta_b,dtheta_b,Tw,Tp],[0,0,0,0,0,0,0,0]);
A = subs(jacobian([dtheta_l,ddtheta_l, ds_w,dds_w,dtheta_b,ddtheta_b],[theta_l,dtheta_l,s_w,ds_w,theta_b,dtheta_b]),[s_w,ds_w,theta_l,dtheta_l,theta_b,dtheta_b,Tw,Tp],[0,0,0,0,0,0,0,0])
B = subs(jacobian([dtheta_l,ddtheta_l, ds_w,dds_w,dtheta_b,ddtheta_b],[Tw,Tp]),[s_w,ds_w,theta_l,dtheta_l,theta_b,dtheta_b,Tw,Tp],[0,0,0,0,0,0,0,0])

vars = [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g];  
vals = [params.Ib, params.Il, params.Iw, params.Mb, params.Ml, params.Mw, params.Lcp, params.Lwc, params.Lpb, params.Rw, params.g];

% format long g
A = double(subs(A, vars, vals));
B = double(subs(B, vars, vals));
C = diag([ 1 1 1 1 1 1]);
D = [
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
];

sys = ss(A, B, C, D);
end