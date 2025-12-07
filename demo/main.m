%% ================== 0. 系统程序 ==================
clc; clear; close all; format long;
rng(7);
%% ================== 1. 参数设置 ==================
N       = 5;                  % 节点数量
K       = 2;                  % 多项式阶数
dt      = 0.02;               % 时间步长 (s)
psi     = N * (N-1) / 2;      % 链路条数
phi     = (K+1) * psi;        % 所有 beta 的维数
T_end   = 200;                % 仿真总时间 (s)
T_seq   = 0:dt:T_end;         % 时间轴
Nsteps  = length(T_seq);      % 仿真步数
c       = 299792458;          % 光速 (m/s)
sigma_t = 1e-7;               % 伪距测量噪声 (秒)
sigma_a = 0.0;                % 速度随机游走  目前不考虑速度随机游走
delta_th = 5e-7;              % 设定 offset操作参数阈值
omega_th = 3.2*1e-12;         % 设定 skew  操作参数阈值
H_max    = 2^2;                 % 设定计数器大小
% q1      = 1e-12;            % 参考论文中的clock WFM
% q2      = 1e-10;            % 参考论文中的clock RWFM
% q1 = 1e-20;                 % Clock 1: white FM 强度  目前认为是s平方
% q2 = 1e-26;                 % Clock 1: RWFM 强度      目前认为是无量纲
% q1      = 1e-12;              % clock WFM  (sigma_theta)^2
% q2      = 1e-13;              % clock RWFM (sigma_gama)^2

q1      = 1e-10;              % clock WFM  (sigma_theta)^2
q2      = 1e-12;              % clock RWFM (sigma_gama)^2




% 时钟初值范围
offset_min = -1e-3; offset_max = 1e-3;
skew_min   =  1e-9 ; skew_max   = 1e-8;

% ====== 初始轨迹（相对坐标） ======
p_idx = 10;
v_idx = 0.01;
position_init_rel = [ 8958,  -1374,  -7,  1935,   38;
    -13326,   2890,   3,   699, 1783;
    9683,  11993, 170,  4248, 3712] * p_idx;

velocity_init_rel = [181, -61, -55, 37, -71;
    -70, 163, 84, 139, 94;
    47,  17, 122, 75, 62]   * v_idx;


%velocity_init_rel = zeros(3,5);


accelerate_init_rel = zeros(3, N);

%% ================== 2. 预生成真实几何轨迹 ==================
pos_hist = zeros(3, N, Nsteps);
vel_hist = zeros(3, N, Nsteps);
position_rel_k   = position_init_rel;
velocity_rel_k   = velocity_init_rel;
accelerate_rel_k = accelerate_init_rel;

for k = 1:Nsteps
    pos_hist(:,:,k) = position_rel_k;
    vel_hist(:,:,k) = velocity_rel_k;
    if k < Nsteps
        [position_rel_k, velocity_rel_k] = pv_update( ...
            position_rel_k, velocity_rel_k, accelerate_rel_k, dt,sigma_a);
    end
end


%% ================== 3. 时钟真值初始化 ==================
[offset_true, skew_true] = offset_skew_init( ...
    offset_min, offset_max, skew_min, skew_max, N);

%真实值给第一个时刻
offset_true_k = offset_true;
skew_true_k   = skew_true;

%% ================== 4. 存储变量 ==================
history_offset_true = zeros(N, Nsteps);
history_skew_true   = zeros(N, Nsteps);
history_offset_est  = zeros(N, Nsteps);
history_skew_est    = zeros(N, Nsteps);
history_dij_true = zeros(N, N, Nsteps);
history_dij_est  = zeros(N, N, Nsteps);
RAMSE_offset_history = zeros(1, Nsteps);
RAMSE_skew_history   = zeros(1, Nsteps);
RAMSE_relativedij_history = zeros(1, Nsteps);
history_offset_true(:,1) = offset_true_k(:);
history_skew_true(:,1)   = skew_true_k(:);
%% ================== 5. KF 初始化 ==================
[L, Q, Xk_1, Hk_1, R, Zk, B] = kalmanInit(N, K, dt, q1, q2, sigma_t);

Z_prev        = zeros(2*psi,1);                      %伪距矩阵缓存
C_prev        = zeros(2*psi,phi+2*N-2);              %观测矩阵缓存
mu_delta_prev = zeros(N-1,1);    %控制offset矩阵缓存
mu_omega_prev = zeros(N-1,1);    %控制skew矩阵缓存

Lambda_prev = [ zeros(phi,1);
    mu_delta_prev;
    mu_omega_prev ];             %初始化控制矩阵

%% ================== 6. 主循环 ==================
tic;
for k = 1:Nsteps
    T_n = T_seq(k);

    %% --- (1) 时钟真值演化 ---


    [offset_true_k, skew_true_k] = offset_skew_update_prediction_control( ...
        offset_true_k, skew_true_k, dt, N, q1, q2, mu_delta_prev, mu_omega_prev);

    history_offset_true(:,k) = offset_true_k(:);
    history_skew_true(:,k)   = skew_true_k(:);


    %% --- (2) 用真实几何轨迹生成伪距（STWR 模型） ---
    pos_hist_k = pos_hist(:,:,k);
    vel_hist_k = vel_hist(:,:,k);
    [rhomat_k, dij_true_mat] = pseudorange_STWR( ...
        N, position_init_rel, velocity_init_rel, ...
        offset_true_k, skew_true_k, T_n, c, sigma_t);

    % [rhomat_k, dij_true_mat] = pseudorange_STWR( ...
    %     N, pos_hist_k, vel_hist_k, offset_true_k, skew_true_k, ...
    %     T_n, c, sigma_t);

    history_dij_true(:,:,k) = dij_true_mat;

    % 转成秒：z = rho / c
    Zk = rhomat_k / c;
    if any(isnan(Zk))
        error("Zk 出现 NaN at step %d", k);
    end
    %% --- (3) KF 预测阶段：每一轮都要做 ---
    if k > 2

        % 估计值存入延迟缓冲区
        % 缓冲区移位 (FIFO)
    end


    [Xkk_1, Hkk_1] = kf_predict(L, Q, Xk_1, Hk_1, B, Lambda_prev);
    % 当前时刻的观测矩阵（对应 Zk）
    Omega_n = Omega_mat_k(N, K, T_n, Zk);
    if any(isnan(Omega_n))
        error("Omega_n 出现 NaN at step %d", k);
    end

    T_nmat = T_mat_k(N);
    if any(isnan(T_nmat))
        error("T_nmat 出现 NaN at step %d", k);
    end

    C_n     = [Omega_n, T_nmat];
    if any(isnan(C_n))
        error("C_n 出现 NaN at step %d", k);
    end

    % --- KF 更新 ---
    [Xk, Hk, ~] = kf_update(Xkk_1, Hkk_1, R, Zk, C_n);


    % 窗口长度（步数）
    M = 200;  % 举例：5 秒窗口，M = 5/0.02

    if k > 2 && mod(k-1, M) == 0
        % 每隔 M 步重置一次 beta
        sigma_beta0 = 1e-8;
        [Xk, Hk] = reset_beta_state(Xk, Hk, phi, sigma_beta0);
        % （时钟相关的部分不动）
    end


    Xk_1 = Xk;
    Hk_1 = Hk;



    %% --- (5) 提取 δ、ω 估计 ---
    [delta_est_k, omega_est_k, history_offset_est, history_skew_est] = ...
        store_clock_est(N, Xk, phi, k, history_offset_est, history_skew_est);


    %%  --- (6) 时钟控制部分 ---
    delta_ctl = delta_est_k(2:N);      % 节点 2..N 的 offset 估计
    omega_ctl = omega_est_k(2:N);      % 节点 2..N 的 skew 估计



    % ----- offset 控制：阈值 + 相位跳变 -----
    mu_delta_k = zeros(N-1,1);
    for i = 1:N-1
        if abs(delta_ctl(i)) > delta_th
            mu_delta_k(i) = delta_ctl(i) * H_max / (2*pi);  % 式(32)
            %mu_delta_k(i) = 1 * delta_ctl(i);
        else
            mu_delta_k(i) = 0;
        end
    end

    % ----- skew 控制：简单 P 控制 + 限幅 -----
    k_p_omega = 1;
    mu_omega_k = k_p_omega * omega_ctl;

    for i = 1:N-1
        if abs(mu_omega_k(i)) > omega_th
            mu_omega_k(i) =  omega_th;
        else
            mu_omega_k(i) =  omega_ctl(i);
        end
    end

    % ----- 组装 Lambda，传给 KF 预测 -----
    Lambda_prev = [ zeros(phi,1);
        mu_delta_k;
        mu_omega_k ];

    % ----- 存下来用于“下一步真值时钟更新” -----
    mu_delta_prev = mu_delta_k;
    mu_omega_prev = mu_omega_k;

    %% --- (7) 计算时钟 RAMSE  ---
    offset_err = history_offset_est(2:N,k) - history_offset_true(2:N,k);
    offset_err = offset_err - mean(offset_err);
    RAMSE_offset_history(k) = sqrt(mean(offset_err.^2));

    skew_err = history_skew_est(2:N,k) - history_skew_true(2:N,k);
    skew_err = skew_err - mean(skew_err);
    RAMSE_skew_history(k) = sqrt(mean(skew_err.^2));


    %% --- (8) 用 beta_hat 估计距离  距离 RAMSE ---
    [history_dij_est, RAMSE_relativedij_history] = ...
        update_distance_and_RAMSE(N, K, c, T_n, Xk, k, ...
        history_dij_est, history_dij_true, RAMSE_relativedij_history);

    if mod(k, 200) == 0
        fprintf("已完成 %d / %d 步\n", k, Nsteps);
    end
end
toc;

[mds_pos_est_hist, RAMSE_pos_history] = ...
    mds_localization_RAMSE(history_dij_est, pos_hist, 1);



%% 作图
set(0,'defaultAxesFontName','Microsoft YaHei');

figure('Position',[100 100 1200 820]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
sgtitle('仿真结果','FontSize',16,'FontWeight','bold');

% offset RAMSE
nexttile;
plot(T_seq, RAMSE_offset_history,'LineWidth',1.8);
title("时钟offset RAMSE"); grid on;

% skew RAMSE
nexttile;
plot(T_seq, RAMSE_skew_history,'LineWidth',1.8);
title("时钟skew RAMSE"); grid on;

% 距离 RAMSE
nexttile;
plot(T_seq, RAMSE_relativedij_history, 'LineWidth', 1.8);
title("相对距离d_ij RAMSE"); grid on;

% MDS 位置 RAMSE
nexttile;
plot(T_seq, RAMSE_pos_history,'LineWidth',1.8);
title("MDS 位置 RAMSE (位置误差)"); grid on;



%% 索引查找项
% a = history_skew_true(:,5003);
% b = history_skew_est(:,500offset3);
% c = history_offset_true(:,2003);
% d = history_offset_est(:,2003);
%% ================== 9. 单个节点 offset / skew 曲线对比 ==================
node_idx = 2;   % 看节点2，也可以改 3,4,5

offset_true_node = history_offset_true(node_idx, :);
offset_est_node  = history_offset_est(node_idx, :);
skew_true_node   = history_skew_true(node_idx, :);
skew_est_node    = history_skew_est(node_idx, :);

figure('Position',[200 200 1000 600]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% offset 对比
nexttile;
plot(T_seq, offset_true_node, 'r', 'LineWidth', 1.5); hold on;
plot(T_seq, offset_est_node,  'g', 'LineWidth', 1.3);
grid on;
xlabel('时间 (s)'); ylabel('offset (s)');
title(sprintf('节点 %d 的 clock offset：真值 vs 估计', node_idx));
legend('True offset', 'Estimated offset', 'Location', 'best');

% skew 对比
nexttile;
plot(T_seq, skew_true_node, 'r', 'LineWidth', 1.5); hold on;
plot(T_seq, skew_est_node,  'g', 'LineWidth', 1.3);
grid on;
xlabel('时间 (s)'); ylabel('skew (s/s)');
title(sprintf('节点 %d 的 clock skew：真值 vs 估计', node_idx));
legend('True skew', 'Estimated skew', 'Location', 'best');
