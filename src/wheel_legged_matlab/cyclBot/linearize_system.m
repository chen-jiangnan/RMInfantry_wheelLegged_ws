% linearize_system_v2.m
% 系统线性化 - 在平衡点附近线性化（符号形式）
%
% 输入: dynamics_new_coords.mat (用广义坐标表示的方程)
% 输出: linearized_system.mat (符号形式的 A, B 矩阵函数)
%
% 对照推导文档 §4.3, §5, §6
%
% ========== 状态向量 (10维, 对应推导 §4.3) ==========
%   X = [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]^T
%   其中:
%     X_b^h    : 机体水平位置 (m)
%     V_b^h    : 机体水平速度 (m/s)
%     phi      : 偏航角 Yaw (rad)
%     dphi     : 偏航角速度 (rad/s)
%     theta_l  : 左腿与Z轴负方向夹角 (rad)
%     dtheta_l : 左腿角速度 (rad/s)
%     theta_r  : 右腿与Z轴负方向夹角 (rad)
%     dtheta_r : 右腿角速度 (rad/s)
%     theta_b  : 机体俯仰角 Pitch (rad)
%     dtheta_b : 机体俯仰角速度 (rad/s)
%
% ========== 控制向量 (4维, 对应推导 §4.3) ==========
%   U = [T_{r→b}, T_{l→b}, T_{wr→r}, T_{wl→l}]^T
%   其中:
%     T_{r→b}  : 右腿对机体的扭矩（右髋关节电机）(Nm)
%     T_{l→b}  : 左腿对机体的扭矩（左髋关节电机）(Nm)
%     T_{wr→r} : 右轮对右腿的扭矩（右轮电机）(Nm)
%     T_{wl→l} : 左轮对左腿的扭矩（左轮电机）(Nm)
%
% 注意：本脚本只输出符号形式，具体参数在 compute_lqr.m 中代入

clear; clc;
fprintf('========================================\n');
fprintf('系统线性化\n');
fprintf('========================================\n\n');

%% =============Step 1: 定义符号变量================
fprintf('Step 1: 定义符号变量...\n\n');

% ========== 广义坐标及其导数 ==========
syms s_w ds_w dds_w real
syms theta_l dtheta_l ddtheta_l real
syms theta_b dtheta_b ddtheta_b real

% ========== 控制力矩 ==========
syms Tw Tp real

% ========== 物理参数 (保持符号形式) ==========
syms g Mw Ml Mb Iw Il Ib Lwc Lcp Lpb Rw


%% =============Step 2: 加载动力学方程===============
fprintf('Step 2: 加载动力学方程...\n');
load('dynamics.mat', 'equs_acc');
fprintf('  已加载 dynamics.mat \n\n');
dds_w = equs_acc.dds_w;
ddtheta_l = equs_acc.ddtheta_l;
ddtheta_b = equs_acc.ddtheta_b;


%% =============Step 3: 构建 6x6 状态空间矩阵===========
fprintf('========================================\n');
fprintf('Step 3:构建 6x6 状态空间 A, B矩阵\n');
fprintf('========================================\n');

% 加速度、速度列表
dx_list = [ ...
    dtheta_l,   ddtheta_l,  ds_w,   dds_w,  dtheta_b,   ddtheta_b];
x_list = [ ...
     theta_l,    dtheta_l,   s_w,    ds_w,   theta_b,    dtheta_b];

% 定义平衡点
eq_point = [ ...
    s_w, ds_w, theta_l, dtheta_l, theta_b, dtheta_b, Tw, Tp];
eq_value = [ ...
    0,   0,     0,        0,        0,        0,      0,  0];

% 定义控制量
u_list = [Tw, Tp];

% 线性化
A = subs(jacobian(dx_list, x_list), eq_point, eq_value);
B = subs(jacobian(dx_list, u_list), eq_point, eq_value);

fprintf('符号 A 矩阵已构建 (6x6)\n');
display(A)
fprintf('符号 B 矩阵已构建 (6x2)\n\n');
display(B)

%% =============Step 4: 平衡点检查================
fprintf('========================================\n');
fprintf('Step 4: 平衡点检查\n');
fprintf('========================================\n');

% 检查加速度在平衡点的值
dds_w_eq     = simplify(subs(dds_w,     eq_point, eq_value));
ddtheta_l_eq = simplify(subs(ddtheta_l, eq_point, eq_value));
ddtheta_b_eq = simplify(subs(ddtheta_b, eq_point, eq_value));

fprintf('平衡点加速度检查结果：\n');
disp('dds_w =');     disp(dds_w_eq)
disp('ddtheta_l ='); disp(ddtheta_l_eq)
disp('ddtheta_b ='); disp(ddtheta_b_eq)

% 是否为严格平衡
if isequal(dds_w_eq, sym(0)) && ...
   isequal(ddtheta_l_eq, sym(0)) && ...
   isequal(ddtheta_b_eq, sym(0))
    fprintf('✓ 该点是严格平衡点\n\n');
else
    fprintf('✗ 该点不是平衡点，需要非零力矩或非零姿态\n\n');
end

%% =============Step 5: 创建 MATLAB 函数句柄===========
fprintf('========================================\n');
fprintf('Step 5:创建 matlabFunction 函数句柄\n');
fprintf('========================================\n\n');

% 参数列表 (左右腿参数分开)
param_list = [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g];

fprintf('参数列表:\n');
fprintf('  [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g]\n\n');

% 转换为函数句柄
fprintf('生成 A_func...\n');
A_func = matlabFunction(A, 'Vars', {param_list});

fprintf('生成 B_func...\n');
B_func = matlabFunction(B, 'Vars', {param_list});

fprintf('✓ 函数句柄创建完成!\n\n');


%% =============Step 6: 保存结果===========
fprintf('========================================\n');
fprintf('Step 6: 保存结果...\n');
fprintf('========================================\n');
save('linearized_system.mat', ...
     'A_func', 'B_func', ...
     'A', 'B',  ...
     'param_list');

fprintf('  结果已保存到 linearized_system.mat\n');
fprintf('\n========================================\n');
fprintf('线性化完成!\n');
fprintf('========================================\n');
