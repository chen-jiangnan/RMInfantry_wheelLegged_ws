function equs_acc = solve_acc(eqs_red)

syms  dds_w  ddtheta_l ddtheta_b
syms f Fls Flh Fbs Fbh

equ_Fbs = solve(eqs_red(6),Fbs);
equ_Fbh = solve(eqs_red(7),Fbh);
equ_ddtheta_b = ddtheta_b == solve(subs(eqs_red(8),[Fbs Fbh],[equ_Fbs equ_Fbh]), ddtheta_b);

equ_Fls = solve(subs(eqs_red(3),Fbs, equ_Fbs), Fls);
equ_Flh = solve(subs(eqs_red(4),Fbh, equ_Fbh), Flh);
equ_ddtheta_l = ddtheta_l == solve(subs(eqs_red(5),[Fls,Flh,Fbs,Fbh],[equ_Fls,equ_Flh,equ_Fbs,equ_Fbh]), ddtheta_l);

equ_f = solve(eqs_red(2),f);
equ_dds_w = dds_w == solve(subs(eqs_red(1),[f,Fls], [equ_f,equ_Fls]), dds_w);

equs_acc = solve(equ_dds_w, equ_ddtheta_l, equ_ddtheta_b, dds_w, ddtheta_l, ddtheta_b);

end