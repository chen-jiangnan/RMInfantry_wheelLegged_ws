% compute_lqr.m
% 基于线性化状态空间模型计算LQR控制器增益矩阵
% 
% 依赖文件: linearized_system.mat (由 linearize_system_v2.m 生成)
%
% ========================================================================
%                          变量定义 (对应推导文档 §4.3)
% ========================================================================
%
% 状态向量 X (10维):
%   X = [X_b^h; V_b^h; phi; dphi; theta_l; dtheta_l; theta_r; dtheta_r; theta_b; dtheta_b]
%
%   序号   符号          物理意义                        单位
%   ─────────────────────────────────────────────────────────────────────
%    1    X_b^h        机体水平位置                      m
%    2    V_b^h        机体水平速度 (= dX_b^h/dt)        m/s
%    3    phi          偏航角                            rad
%    4    dphi         偏航角速度                        rad/s
%    5    theta_l      左腿与Z轴负方向夹角               rad
%    6    dtheta_l     左腿角速度                        rad/s
%    7    theta_r      右腿与Z轴负方向夹角               rad
%    8    dtheta_r     右腿角速度                        rad/s
%    9    theta_b      机体俯仰角                        rad
%   10    dtheta_b     机体俯仰角速度                    rad/s
%
% 控制向量 u (4维):
%   u = [T_{r→b}; T_{l→b}; T_{wr→r}; T_{wl→l}]
%
%   序号   符号          物理意义                        执行器
%   ─────────────────────────────────────────────────────────────────────
%    1    T_{r→b}      右腿对机体的扭矩                  右髋关节电机
%    2    T_{l→b}      左腿对机体的扭矩                  左髋关节电机
%    3    T_{wr→r}     右轮对右腿的扭矩                  右轮电机
%    4    T_{wl→l}     左轮对左腿的扭矩                  左轮电机
%
% 扭矩符号约定: T_{A→B} 表示物体A对物体B施加的扭矩
%
% LQR控制律:
%   u = -K * X
%
% 其中 K 为 4×10 增益矩阵
%
% ========================================================================
% 作者: 基于2026公式推导
% 日期: 2026/01/13
% ========================================================================

clear all; clc;
tic

%% ======================== Step 0: 加载线性化系统 ========================
fprintf('========================================\n');
fprintf('轮腿机器人LQR控制器计算\n');
fprintf('========================================\n\n');

fprintf('Step 0: 加载线性化状态空间模型...\n');

% 检查文件是否存在
if ~exist('linearized_system.mat', 'file')
    error('未找到 linearized_system.mat! 请先运行 linearize_system_v2.m');
end

load('linearized_system.mat', 'A_func', 'B_func', 'A', 'B', 'param_list');

fprintf('  ✓ 线性化系统加载成功\n');

% 获取函数句柄的输出维度 (22 个参数)
% param_list = param_list = [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g];
test_params = [1, 1, 1, 0.2, 0.2, 0.1, 0.03, 0.03, 0.0002, 0.0002, 0.4, 0.2, 0.2, 0.084, 0.084, 0.05, 0.055, 0.1, -9.81, 0.1, 0.1, 0.05];
A_test = A_func(test_params);
B_test = B_func(test_params);
fprintf('  状态维度: %d\n', size(A_test, 1));
fprintf('  控制维度: %d\n', size(B_test, 2));


%% ======================== Step 1: 定义物理参数 ========================
fprintf('========================================\n');
fprintf('Step 1: 定义机器人物理参数...\n');
fprintf('========================================\n\n');

% 几何参数
Rw_vel  = 0.135/2;          % R_w   驱动轮半径                         [m]
Dw_vel  = 0.135;            % D_w   驱动轮间距/2                       [m]
Ll_vel  = 0.20;             % L_l   虚拟腿长度                         [m]
Lwc_vel = (1/7)*Ll_vel;     % L_wc  轮质心 → 虚拟腿质心                 [m]
Lcp_vel = (6/7)*Ll_vel;     % L_cp  虚拟腿质心 → 髋关节                 [m]
Lpb_vel = 0.022962;         % L_pb  髋关节 → 机体质心                   [m]

% 质量参数
Mw_vel = 1.2451;            % M_w   驱动轮质量                         [kg]
Ml_vel = 0.9517;            % M_l   虚拟腿质量                         [kg]
Mb_vel = 20.36;             % M_b   机体质量                           [kg]w

% 转动惯量
Iw_vel = 0.00112195;                            % I_w   驱动轮转动惯量       [kg·m^2]
Il_vel = 0.015;                                 % I_l   虚拟腿转动惯量       [kg·m^2]
Ib_vel = Mb_vel*Lpb_vel^2 + Mb_vel*(0.2^2+0.1^2)/12;        % I_b   机体x轴转动惯量      [kg·m^2]
Iz_vel = 0;                                     % I_z   机体z轴转动惯量      [kg·m^2]

% 环境参数
g_vel  = 9.80665;           % g     重力加速度                         [m/s^2]

fprintf('  ✓ 物理参数设置完成\n\n');

%% ======================== Step 2: 数值代入 ========================
fprintf('========================================\n');
fprintf('Step 2: 代入数值参数...\n');
fprintf('========================================\n\n');

% 参数值向量 (顺序与 linearize_system_v2.m 中 param_list 一致)
% param_list = param_list = [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g];
param_vals = [Ib_vel, Il_vel, Iw_vel, Mb_vel, Ml_vel, Mw_vel, Lcp_vel, Lwc_vel, Lpb_vel, Rw_vel, g_vel];

% 使用函数句柄计算数值矩阵
A_num = A_func(param_vals);
B_num = B_func(param_vals);

fprintf('  ✓ 数值代入完成\n');

% 显示矩阵
fprintf('\n  数值A矩阵 (6×6):\n');
disp(A_num);
fprintf('  数值B矩阵 (6×2):\n');
disp(B_num);


%% ======================== Step 3: 检查可控性 ========================
fprintf('========================================\n');
fprintf('Step 3: 检查系统可控性...\n');
fprintf('========================================\n\n');

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('  可控性矩阵秩: %d (系统维度: 6)\n', rank_Co);

if rank_Co < 6
    fprintf('\n  ⚠ 系统不完全可控 (秩=%d < 10)\n', rank_Co);
    fprintf('  物理原因: X_b^h(水平位置) 和 phi(yaw角) 是积分器状态\n');
    fprintf('           机器人可以在任意位置/朝向平衡，这两个状态不影响动力学\n');
    fprintf('  解决方案: 这是正常的! LQR仍然可以计算可控子空间的增益\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end


%% ======================== Step 4: 设置LQR权重 ========================
fprintf('========================================\n');
fprintf('Step 4: 设置LQR权重矩阵...\n');
fprintf('========================================\n\n');

% Q矩阵: 状态权重
% 状态: [theat_l, dtheta_l, s_w, ds_w,  theta_b, dtheta_b]
%         腿角      腿速     位置  速度     俯仰角   俯仰速
lqr_Q = diag( [1 1 500 100 5000 1]);

% R矩阵: 控制输入权重
% 控制: [ Tw,       Tp]
%     驱动轮力矩    腿力矩
lqr_R = diag([1 0.25]);

fprintf('  Q矩阵 (状态权重):\n');
fprintf('         [theat_l, dtheta_l,      s_w,       ds_w,     theta_b,    dtheta_b]\n');
disp(lqr_Q);

fprintf('  R矩阵 (控制权重):\n');
fprintf('    [ Tw,       Tp ]\n');
disp(lqr_R);



%% ======================== Step 5: 计算LQR增益 ========================
fprintf('========================================\n');
fprintf('Step 5: 计算LQR增益矩阵K...\n');
fprintf('========================================\n\n');


try
    [K, S, e] = lqr(A_num, B_num, lqr_Q, lqr_R);
    
    fprintf('  ✓ LQR增益矩阵K (2x6):\n');
    disp(K);
    
    fprintf('  闭环特征值:\n');
    disp(e);
    
    % 检查稳定性
    stable_eigs = real(e) < 1e-6;
    if all(stable_eigs)
        fprintf('  ✓ 闭环系统稳定!\n\n');
    else
        warning('闭环系统不稳定!');
    end
catch ME
    fprintf('  ✗ LQR计算失败: %s\n', ME.message);
    K = [];
end

%% ======================== Step 6: 格式化输出 ========================
fprintf('========================================\n');
fprintf('格式化输出 (可直接复制到C代码)\n');
fprintf('========================================\n\n');

if ~isempty(K)
    % K矩阵行列含义
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// LQR增益矩阵 K[2][6]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 控制律: u = -K * X\n');
    fprintf('//\n');
    fprintf('// 状态向量 X (列向量 6×1):\n');
    fprintf('//   X[0] =  theta_l  摆杆角度 (rad)\n');
    fprintf('//   X[1] = dtheta_l  摆杆角速度 (rad/s)\n');
    fprintf('//   X[2] =  s_w      机体水平位置 (m)\n');
    fprintf('//   X[3] = ds_w      机体水平速度 (m/s)\n');
    fprintf('//   X[4] =  theta_b  机体俯仰角 (rad)\n');
    fprintf('//   X[5] = dtheta_b  机体俯仰角速度 (rad/s)\n');
    fprintf('//\n');
    fprintf('// 控制向量 u (列向量 2×1):\n');
    fprintf('//   u[0] = Tw   驱动轮扭矩 (Nm)\n');
    fprintf('//   u[1] = Tp   摆杆扭矩(Nm)\n');
    fprintf('//\n');
    fprintf('// K矩阵含义:\n');
    fprintf('//   K[i][j] 表示控制输入 u[i] 对状态 X[j] 的反馈增益\n');
    fprintf('//   K[0][*]: 驱动轮扭矩对各状态的增益\n');
    fprintf('//   K[1][*]: 摆杆扭矩对各状态的增益\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

    fprintf('float K[2][6] = {\n');
    fprintf('float K[2][6] = {\n');

    control_names = {'Tw', 'Tp'};
    for i = 1:2
        fprintf('    {');
        for j = 1:6
            if j < 6
                fprintf('%11.6ff, ', K(i,j));
            else
                fprintf('%11.6ff',  K(i,j));
            end
        end

        if i < 2
            fprintf('},  // %s\n', control_names{i});
        else
            fprintf('}   // %s\n', control_names{i});
        end
    end

fprintf('};\n\n');

    fprintf('// 单行格式 (每行对应一个控制输入):\n');
    for i = 1:2
        fprintf('// K[%d] (%s): ', i-1, control_names{i});
        for j = 1:6
            if j < 6
                fprintf('%.6g, ', K(i,j));
            else
                fprintf('%.6g', K(i,j));
            end
        end
        fprintf('\n');
    end

    fprintf('\n');
    fprintf('\n');
end


%% ======================== Step 7: 腿长拟合功能 ========================
fprintf('========================================\n');
fprintf('Step 7: 腿长拟合功能\n');
fprintf('========================================\n\n');

% 腿长参数查找表
% 格式: [腿长(m), 质心到轮轴距离(m), 质心到髋关节距离(m), 转动惯量(kg·m²)]
Leg_data = [
    0.13, 0.13680373277071062, 0.06482330830804611, 0.01298843908;
    0.14, 0.14177326863693313, 0.06550465403312959, 0.01309498073;
    0.15, 0.14696847451069225, 0.06629277864141764, 0.01320078273;
    0.16, 0.15233205965915383, 0.06716142047336403, 0.01330738064;
    0.17, 0.15785793169809365, 0.06810379284592012, 0.013415874730;
    0.18, 0.16353393531619057, 0.06910968094268703, 0.01352706998;
    0.19, 0.16932790703247946, 0.07018361703417686, 0.0136415662890;
    0.20, 0.17523986903670066, 0.07130926798109767, 0.01375981819;
    0.21, 0.18125438063671734, 0.07249103737704407, 0.0138821752200;
    0.22, 0.18735804573062775, 0.07371456640312009, 0.01400890984;
    0.23, 0.1935440737919919,  0.07498472177717272, 0.01414023708;
    0.24, 0.19981164355462372, 0.07629346564418214, 0.01427632855;
    0.25, 0.20613804355334314, 0.07763950669601141, 0.01441732272;
    0.26, 0.2125254867068889,  0.07902583438344704, 0.0145633324100;
    0.27, 0.21897323717751446, 0.08044798692322885, 0.0147144504;
    0.28, 0.22546613138118995, 0.08189613177678175, 0.01487075369;
    0.29, 0.23200655335571885, 0.08337530089900726, 0.0150323068;
    0.30, 0.23859429875837357, 0.08488368158839482, 0.01519916419;
    0.31, 0.2452153170175142,  0.08641615416112892, 0.01537137233;
    0.32, 0.2518818852160671,  0.0879709275840604,  0.01554897121;
    0.33, 0.2585709469371994,  0.08955408756723503, 0.01573199564;
    0.34, 0.2652931949749182,  0.09115963635293857, 0.01592047625;
    0.35, 0.2720462993683244,  0.09278032657842933, 0.01611444043;
    0.36, 0.27883182906547804, 0.09442027801272353, 0.0163139131;
    0.37, 0.2856372146622355,  0.09608027060744573, 0.01651891744;
    0.38, 0.29246232389831,    0.0977599657323999,  0.0167294757;
    0.39, 0.29930908322334626, 0.09946520647945191, 0.01694561016;
    0.40, 0.30618331143287353, 0.10117420718740523, 0.01716734433;
];

enable_fitting = true;  % 设为 false 跳过腿长拟合

if enable_fitting
    fprintf('正在计算不同腿长下的K矩阵...\n');

    % ========== 计算采样点 (二维网格) ==========
    num_legs = size(Leg_data, 1);
    sample_size_1d = num_legs^1;

    % K矩阵 2×6 = 12 个元素
    % 二维拟合: [Ll, K矩阵的12个元素]
    K_sample_1d = zeros(sample_size_1d, 13);  % [Ll, K矩阵的12个元素]

    tic_fit = tic;

    idx = 0;
    for i = 1:num_legs
        idx = idx + 1;

        % 摆杆参数
        Ll_fit = Leg_data(i, 1);
        Lwc_fit = Leg_data(i, 2);
        Lcp_fit = Leg_data(i, 3);
        Il_fit = Leg_data(i, 4);


        % 构建参数向量
        % param_list = param_list = [Ib, Il, Iw, Mb, Ml, Mw, Lcp, Lwc, Lpb, Rw, g];
        param_fit = [Ib_vel, Il_fit, Iw_vel, Mb_vel, Ml_vel, Mw_vel, Lcp_fit, Lwc_fit, Lpb_vel, Rw_vel, g_vel];

        % 计算数值矩阵
        A_fit = A_func(param_fit);
        B_fit = B_func(param_fit);

        % 计算LQR
        try
            K_fit = lqr(A_fit, B_fit, lqr_Q, lqr_R);

            % 存储结果
            K_sample_1d(idx, 1) = Ll_fit;
            K_sample_1d(idx, 2:13) = K_fit(:)';  % 按行展开
        catch
            warning('LQR计算失败: Ll=%.2f', Ll_fit);
        end

        % 显示进度
        if mod(idx, 49) == 0
            fprintf('  进度: %d/%d (%.1f秒)\n', idx, sample_size_1d, toc(tic_fit));
        end
    end

    fprintf('  ✓ %d 个样本计算完成! 耗时: %.2f秒\n', sample_size_1d, toc(tic_fit));


    % ========== 一维多项式拟合 ==========
    fprintf('\n正在进行一维多项式拟合...\n');

    % 拟合多项式: K_ij(Ll) = p1*Ll^2  + p2*L2 + p3
    K_Fit_Coefficients = zeros(12, 3);

    Ll_samples = K_sample_1d(:, 1);
   
    for n = 1:12
        K_values = K_sample_1d(:, n+1);
        try
            % 一维二次多项式拟合
            K_Surface_Fit = fit(Ll_samples, K_values, 'poly2');
            coeffs = coeffvalues(K_Surface_Fit);
            K_Fit_Coefficients(n, :) = coeffs;  % [p1, p2, p3]
        catch
            warning('一维拟合失败: 元素 %d', n);
        end
    end

    fprintf('  ✓ 拟合完成\n\n');
    % ======================== 曲线显示 ========================
    figure('Name','LQR系数随腿长变化','NumberTitle','off');

    leg_lengths = K_sample_1d(:,1);  % 腿长采样点

    % K_Fit_Coefficients 已经拟合完成 (12x3)
    % Q 和 R 对角元素
    Q_diag = diag(lqr_Q);
    R_diag = diag(lqr_R);

    for n = 1:12
        subplot(3,4,n);  % 3行4列子图
        hold on;

        % 原始样本曲线
        plot(leg_lengths, K_sample_1d(:, n+1), 'bo-', 'LineWidth',1.5, 'MarkerSize',4);

        % 拟合曲线
        % 拟合多项式:  p1*Ll^2 + p2*Ll + p3
        p = K_Fit_Coefficients(n,:);
        Ll_fit = linspace(min(leg_lengths), max(leg_lengths), 100);
        K_fit_curve = p(3) + p(2)*Ll_fit + p(1)*(Ll_fit.^2);
        plot(Ll_fit, K_fit_curve, 'r-', 'LineWidth',1.5);

        grid on;
        xlabel('腿长 L_l (m)');
        ylabel(sprintf('K(%d,%d)', ceil(n/6), mod(n-1,6)+1));  % 上标 K(i,j)
        title(sprintf('K(%d,%d) vs 腿长', ceil(n/6), mod(n-1,6)+1));
    end
    % ===== 在 figure 右上角统一显示 Q / R =====
    str_qr = sprintf('Q = [%s]\nR = [%s]', ...
                    num2str(Q_diag','%g '), ...
                    num2str(R_diag','%g '));

    annotation('textbox', [0.8 0.8 0.26 0.2], ...
        'String', str_qr, ...
        'FitBoxToText','on', ...
        'BackgroundColor','w', ...
        'EdgeColor','k', ...
    'FontSize',9);
    sgtitle('LQR K矩阵元素随腿长变化曲线');
    fprintf('曲线显示完成！\n');


    % ========== 输出拟合系数 ==========
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 腿长拟合系数 K_Fit_Coefficients[12][3]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 拟合多项式 (一维):\n');
    fprintf('//   K_ij(Ll) = p1*Ll^2  + p2*L2 + p3\n');
    fprintf('//\n');
    fprintf('// K元素排列 (按行优先):\n');
    fprintf('//   n=0~5:   K[0][0~5] → Tw 对各状态的增益\n');
    fprintf('//   n=6~15:  K[1][0~5] → Tp 对各状态的增益\n');
    fprintf('//\n');
    fprintf('// 系数顺序: [p1 p2 p3]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

    fprintf('float K_Fit_Coefficients[12][3] = {\n');
    for n = 1:12
        row = ceil(n/6) - 1;     % 0-indexed
        col = mod(n-1, 6);       % 0-indexed

        fprintf('    {');
        for c = 1:3
            if c < 3
                fprintf('%12.6gf, ', K_Fit_Coefficients(n,c));
            else
                fprintf('%12.6gf',  K_Fit_Coefficients(n,c));
            end
        end

        if n < 12
            fprintf('},  // K[%d][%d]\n', row, col);
        else
            fprintf('}   // K[%d][%d]\n', row, col);
        end
    end
    fprintf('};\n\n');
% 
%     % 保存拟合结果
%     save('lqr_fitting_results.mat', 'K_sample_2d', 'K_Fit_Coefficients', 'Leg_data');
%     fprintf('拟合结果已保存到 lqr_fitting_results.mat (左右腿独立参数版本)\n');
end
% 
% %% ======================== Step 8: 保存结果 ========================
% 
% fprintf('\n========================================\n');
% fprintf('保存结果\n');
% fprintf('========================================\n\n');
% 
% save('lqr_results.mat', 'A_num', 'B_num', 'K', 'lqr_Q', 'lqr_R', 'e');
% fprintf('✓ LQR结果已保存到 lqr_results.mat\n');
% 
% elapsed_time = toc;
% fprintf('\n计算完成! 总耗时: %.2f秒\n', elapsed_time);
