function eqs = newton_euler()

syms s_w ds_w dds_w 
syms theta_l dtheta_l ddtheta_l
syms theta_b dtheta_b ddtheta_b

syms f Fls Flh Fbs Fbh
syms Tw Tp

syms g Mw Ml Mb Iw Il Ib Lwc Lcp Lpb Rw

%%ddsin(theta) = +[ ddtheta*cos(theta) - (dtheta)^2*sin(theta) ]
%%ddcos(theta) = -[ ddtheat*sin(theta) + (dtheta)^2*cos(theta) ]

% 驱动轮
eq1 = Mw*dds_w == f - Fls;
eq2 = Iw*dds_w/Rw == Tw - f*Rw;

% 摆杆
eq3 = Fls - Fbs == Ml*(dds_w +  Lwc*( ddtheta_l*cos(theta_l) - (dtheta_l)^2*sin(theta_l) ));
eq4 = Flh - Fbh - Ml*g == Ml*( -Lwc*( ddtheta_l*sin(theta_l) + (dtheta_l)^2*cos(theta_l) ));
eq5 = Il*ddtheta_l == ...
      (Flh*Lwc + Fbh*Lcp)*sin(theta_l) ...
    - (Fls*Lwc + Fbs*Lcp)*cos(theta_l) ...
    - Tw + Tp;

% 机体
eq6 = Fbs == Mb*(dds_w + (Lwc+Lcp)*(  ddtheta_l*cos(theta_l) - (dtheta_l)^2*sin(theta_l) ) ...
                              +Lpb*(  ddtheta_b*cos(theta_b) - (dtheta_b)^2*sin(theta_b) ));

eq7 = Fbh - Mb*g == Mb*( (Lwc+Lcp)*( -ddtheta_l*sin(theta_l) - (dtheta_l)^2*cos(theta_l) ) ...
                              +Lpb*( -ddtheta_b*sin(theta_b) - (dtheta_b)^2*cos(theta_b) ));

eq8 = Ib*ddtheta_b == Fbh*Lpb*sin(theta_b) - Fbs*Lpb*cos(theta_b) - Tp;

% eq1 = Mw*dds_w == f - Fls;
% eq2 = Iw*dds_w/Rw == Tw - f*Rw;
% % sin(theta)->theta_dot*cos(theta)->theta_ddot*cos(theta)-theta_dot*theta_dot*sin(theta)
% % cos(theta)->-theta_dot*sin(theta)->-theta_ddot*sin(theat)-theta_dot*theta_dot*cos(theta)
% eq3 = Fls - Fbs == Ml*(dds_w+Lwc*(ddtheta_l*cos(theta_l)-dtheta_l*dtheta_l*sin(theta_l)));
% eq4 = Flh - Fbh -Ml*g == Ml*(Lwc*(-ddtheta_l*sin(theta_l)-dtheta_l*dtheta_l*cos(theta_l)));
% eq5 = Il*ddtheta_l == (Flh*Lwc + Fbh*Lcp)*sin(theta_l)-(Fls*Lwc+Fbs*Lcp)*cos(theta_l)-Tw+Tp;
% 
% % 对机体，有：
% eq6 = Fbs == Mb*(dds_w+(Lwc+Lcp)*(ddtheta_l*cos(theta_l)-dtheta_l*dtheta_l*sin(theta_l)) - Lpb*(ddtheta_b*cos(theta_b)-dtheta_b*dtheta_b*sin(theta_b)));
% eq7 = Fbh-Mb*g == Mb*((Lwc+Lcp)*(  -ddtheta_l*sin(theta_l)-dtheta_l*dtheta_l*cos(theta_l)) + Lpb*(-ddtheta_b*sin(theta_b)-dtheta_b*dtheta_b*cos(theta_b)));
% eq8 = Ib*ddtheta_b == Tp + Fbs*Lpb*cos(theta_b) + Fbh*Lpb*sin(theta_b);


eqs = [eq1;eq2;eq3;eq4;eq5;eq6;eq7;eq8];

end