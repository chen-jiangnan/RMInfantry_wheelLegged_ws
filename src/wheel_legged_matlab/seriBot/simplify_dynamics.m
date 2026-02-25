% simplify_dynamics.m
% 动力学方程化简 - 消去内力，得到3个最终方程
% 
% 输入: 原始动力学方程（牛顿-欧拉法）
% 输出: dynamics.mat (3个化简后的动力学方程)

clear; clc;
fprintf('========================================\n');
fprintf('动力学方程化简 (消去内力)\n');
fprintf('========================================\n\n');

%% 定义符号变量

% 物理常数
syms g real

% 状态变量以及各阶导数
syms s_w ds_w dds_w real
syms theta_l dtheta_l ddtheta_l real
syms theta_b dtheta_b ddtheta_b real

% 力/力矩
syms f Fls Flh Fbs Fbh real
syms Tw Tp real

% 系统参数
syms Mw Ml Mb Iw Il Ib Lwc Lcp Lpb Rw real

%% 建立原始动力学方程

fprintf('Step 1: 建立原始动力学方程...\n');

% 驱动轮方程 (2个)
eq1 = Mw*dds_w == f - Fls;
eq2 = Iw*dds_w/Rw == Tw - f*Rw;

% 摆杆方程 (3个)
eq3 = Fls - Fbs == Ml*(dds_w +  Lwc*( ddtheta_l*cos(theta_l) - (dtheta_l)^2*sin(theta_l) ));
eq4 = Flh - Fbh - Ml*g == Ml*( -Lwc*( ddtheta_l*sin(theta_l) + (dtheta_l)^2*cos(theta_l) ));
eq5 = Il*ddtheta_l == ...
      (Flh*Lwc + Fbh*Lcp)*sin(theta_l) ...
    - (Fls*Lwc + Fbs*Lcp)*cos(theta_l) ...
    - Tw + Tp;

% 机体方程 (3个)
eq6 = Fbs == Mb*(dds_w + (Lwc+Lcp)*(  ddtheta_l*cos(theta_l) - (dtheta_l)^2*sin(theta_l) ) ...
                              -Lpb*(  ddtheta_b*cos(theta_b) - (dtheta_b)^2*sin(theta_b) ));

eq7 = Fbh - Mb*g == Mb*( (Lwc+Lcp)*( -ddtheta_l*sin(theta_l) - (dtheta_l)^2*cos(theta_l) ) ...
                              +Lpb*( -ddtheta_b*sin(theta_b) - (dtheta_b)^2*cos(theta_b) ));

eq8 = Ib*ddtheta_b == Fbh*Lpb*sin(theta_b) + Fbs*Lpb*cos(theta_b) + Tp;

eqs = [eq1;eq2;eq3;eq4;eq5;eq6;eq7;eq8];

fprintf('  共 8 个原始方程\n\n');

%% 消去内力

fprintf('Step 2: 消去内力...\n');

% 求解腿对机体水平和竖直方向的力
equ_Fbs = solve(eqs(6),Fbs);
equ_Fbh = solve(eqs(7),Fbh);
% 求解机体theta角加速度方程
equ_ddtheta_b = ddtheta_b == solve(subs(eqs(8),[Fbs Fbh],[equ_Fbs equ_Fbh]), ddtheta_b);

% 求解驱动轮对腿水平方向和竖直方向的力
equ_Fls = solve(subs(eqs(3),Fbs, equ_Fbs), Fls);
equ_Flh = solve(subs(eqs(4),Fbh, equ_Fbh), Flh);
% 求解腿部theta角加速度方程
equ_ddtheta_l = ddtheta_l == solve(subs(eqs(5),[Fls,Flh,Fbs,Fbh],[equ_Fls,equ_Flh,equ_Fbs,equ_Fbh]), ddtheta_l);

% 求解驱动轮所受摩擦力
equ_f = solve(eqs(2),f);
% 求解驱动轮平动加速度方程
equ_dds_w = dds_w == solve(subs(eqs(1),[f,Fls], [equ_f,equ_Fls]), dds_w);

equs_acc = solve(equ_dds_w, equ_ddtheta_l, equ_ddtheta_b, dds_w, ddtheta_l, ddtheta_b);

fprintf('  得到 3 个最终方程\n\n');

%% 打印简化结果

fprintf('Step 3: 打印简化后的方程...\n\n');
fprintf('----------------------------------------\n');
fprintf('方程1: 驱动轮平动加速度方程\n');
fprintf('----------------------------------------\n');
disp(equs_acc.dds_w);
fprintf('----------------------------------------\n');
fprintf('方程2: 摆杆转动方程\n');
fprintf('----------------------------------------\n');
disp(equs_acc.ddtheta_l);
fprintf('----------------------------------------\n');
fprintf('方程3: 右腿转动方程\n');
fprintf('----------------------------------------\n');
disp(equs_acc.ddtheta_b);


%% 保存结果

fprintf('Step 4: 保存结果...\n');

save('dynamics.mat', 'equs_acc');

fprintf('  结果已保存到 dynamics.mat\n');
fprintf('\n========================================\n');
fprintf('动力学方程化简完成!\n');
fprintf('========================================\n');
