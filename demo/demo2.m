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
sigma_a = 0.0;                % 速度随机游走 目前不考虑

% ---- 控制相关参数 ----
delta_th = 5e-7;              % offset 操作阈值
omega_th = 3.2e-12;           % skew  操作限幅 (Eq.34 对应)
H_max    = 2;                 % 计数器大小（简化）

% ---- 时钟噪声参数（你可以按项目默认 / 论文调）----
% 项目统一设定（你之前说的）：
% q1 = 1e-12;                 % WFM
% q2 = 1e-16;                 % RWFM

q1 = 1e-8;                    % WFM
q2 = 1.44e-10;                %  RWFM

% ---- 时钟初始范围 ----
offset_min = -1e-3; offset_max = 1e-3;
skew_min   =  1e-5; skew_max   = 1e-4;

%% ====== 初始轨迹（相对坐标） ======
p_idx = 10;
v_idx = 0.01;
position_init_rel = [ 8958,  -1374,  -7,  1935,   38;
                     -13326,  2890,   3,   699, 1783;
                      9683, 11993, 170,  4248, 3712] * p_idx;

velocity_init_rel = [181,  -61,  -55,   37,  -71;
                     -70,  163,   84,  139,   94;
                      47,   17,  122,   75,   62] * v_idx;

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
            position_rel_k, velocity_rel_k, accelerate_rel_k, dt, sigma_a);
    end
end

%% ================== 3. 时钟真值初始化 ==================
[offset_true, skew_true] = offset_skew_init( ...
    offset_min, offset_max, skew_min, skew_max, N);

offset_true_k = offset_true;
skew_true_k   = skew_true;


%% ================== 4. 存储变量 ==================
history_offset_true = zeros(N, Nsteps);
history_skew_true   = zeros(N, Nsteps);
history_offset_est  = zeros(N, Nsteps);
history_skew_est    = zeros(N, Nsteps);

history_dij_true = zeros(N, N, Nsteps);
history_dij_est  = zeros(N, N, Nsteps);

RAMSE_offset_history      = zeros(1, Nsteps);
RAMSE_skew_history        = zeros(1, Nsteps);
RAMSE_relativedij_history = zeros(1, Nsteps);

% 第一个时刻的真值
history_offset_true(:,1) = offset_true_k(:);
history_skew_true(:,1)   = skew_true_k(:);

%% ================== 5. KF 初始化 ==================
[L, Q, Xk_1, Hk_1, R, Zk_init, B] = kalmanInit(N, K, dt, q1, q2, sigma_t);

Z_prev        = zeros(2*psi,1);                % 伪距缓存（暂未用）
C_prev        = zeros(2*psi,phi+2*N-2);        % 观测矩阵缓存（暂未用）
mu_delta_prev = zeros(N-1,1);                  % 上一轮 offset 控制
mu_omega_prev = zeros(N-1,1);                  % 上一轮 skew   控制

Lambda_prev = [ zeros(phi,1);
                mu_delta_prev;
                mu_omega_prev ];               % 控制向量

% ====== 延迟缓冲区：实现 n → n+2 延迟控制 ======
delay_steps = 2;
est_buffer = cell(delay_steps + 1, 1);         % FIFO 缓冲区

%% ================== 6. 主循环 ==================
tic;
M = 200;   % 多项式窗口长度（步数）

for k = 1:Nsteps
    T_n = T_seq(k);

    %% --- (1) 时钟真值演化（物理 + 控制） ---
    if k > 1
        [offset_true_k, skew_true_k] = offset_skew_update_prediction_control( ...
            offset_true_k, skew_true_k, dt, N, q1, q2, ...
            mu_delta_prev, mu_omega_prev);
        history_offset_true(:,k) = offset_true_k(:);
        history_skew_true(:,k)   = skew_true_k(:);
    end

    %% --- (2) 用真实轨迹 + 时钟真值生成伪距（STWR 模型） ---
    pos_hist_k = pos_hist(:,:,k);
    vel_hist_k = vel_hist(:,:,k);

    [rhomat_k, dij_true_mat] = pseudorange_STWR( ...
        N, position_init_rel, velocity_init_rel, ...
        offset_true_k, skew_true_k, T_n, c, sigma_t);

    history_dij_true(:,:,k) = dij_true_mat;

    % 转成秒：z = rho / c
    Zk = rhomat_k / c;
    if any(isnan(Zk))
        error("Zk 出现 NaN at step %d", k);
    end

    %% --- (3) KF 预测 ---
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

    C_n = [Omega_n, T_nmat];
    if any(isnan(C_n))
        error("C_n 出现 NaN at step %d", k);
    end

    %% --- (4) KF 更新 ---
    [Xk, Hk, ~] = kf_update(Xkk_1, Hkk_1, R, Zk, C_n);

    % 多项式窗口重置（避免长时间拟合导致 β 发散）
    if k > 2 && mod(k-1, M) == 0
        sigma_beta0 = 1e-8;  % 你可以继续调
        [Xk, Hk] = reset_beta_state(Xk, Hk, phi, sigma_beta0);
    end

    Xk_1 = Xk;
    Hk_1 = Hk;

    %% --- (5) 提取 δ、ω 估计，并存历史 ---
    [delta_est_k, omega_est_k, history_offset_est, history_skew_est] = ...
        store_clock_est(N, Xk, phi, k, history_offset_est, history_skew_est);

    %% --- (6) 把当前估计存入延迟缓冲区 (对应时刻 n 的估计) ---
    current_est.offset = delta_est_k;   % N×1
    current_est.skew   = omega_est_k;   % N×1

    % FIFO 左移
    est_buffer(1:end-1) = est_buffer(2:end);
    est_buffer{end} = current_est;

    %% --- (7) 时钟控制部分：无预测 + 两步延迟 (n → n+2) ---
    mu_delta_k = zeros(N-1,1);
    mu_omega_k = zeros(N-1,1);

    % 只有当缓冲区已经“装满”足够多步数后，才使用延迟控制
    if k > delay_steps + 0
        delayed = est_buffer{1};    % 两步之前的估计

        if ~isempty(delayed)
            % 节点2..N的误差
            delta_base = delayed.offset(2:N);
            omega_base = delayed.skew(2:N);

            % ======= 无预测控制：直接使用两步前的误差 =======
            delta_target = delta_base;
            omega_target = omega_base;
            % ===============================================

            % --- offset 控制：阈值 + 相位跳变 (式(32)) ---
            for i = 1:N-1
                if abs(delta_target(i)) > delta_th
                    mu_delta_k(i) = delta_target(i) * H_max / (2*pi);
                    % 或者你可以先用调试版：mu_delta_k(i) = delta_target(i);
                else
                    mu_delta_k(i) = 0;
                end
            end

            % --- skew 控制：P 控制 + 限幅 (式(34) 对应) ---
            k_p_omega = 1;
            mu_omega_raw = k_p_omega * omega_target;

            % for i = 1:N-1
            %     if abs(mu_omega_raw(i)) > omega_th
            %         mu_omega_k(i) = sign(mu_omega_raw(i)) * omega_th;
            %     else
            %         mu_omega_k(i) = mu_omega_raw(i);
            %     end
            % end
        end
    end

    % 组装控制向量 Lambda，给下一次 KF 预测用
    Lambda_prev = [ zeros(phi,1);
                    mu_delta_k;
                    mu_omega_k ];

    % 保存控制量，给下一步“真实时钟演化”使用
    mu_delta_prev = mu_delta_k;
    mu_omega_prev = mu_omega_k;

    %% --- (8) 计算时钟 RAMSE  ---
    if k >= 1
        offset_err = history_offset_est(2:N,k) - history_offset_true(2:N,k);
        offset_err = offset_err - mean(offset_err);
        RAMSE_offset_history(k) = sqrt(mean(offset_err.^2));

        skew_err = history_skew_est(2:N,k) - history_skew_true(2:N,k);
        skew_err = skew_err - mean(skew_err);
        RAMSE_skew_history(k) = sqrt(mean(skew_err.^2));
    
    end
    %% --- (9) 用 beta_hat 估计距离 & 距离 RAMSE ---
    [history_dij_est, RAMSE_relativedij_history] = ...
        update_distance_and_RAMSE(N, K, c, T_n, Xk, k, ...
        history_dij_est, history_dij_true, RAMSE_relativedij_history);
 
    if mod(k, 200) == 0
        fprintf("已完成 %d / %d 步\n", k, Nsteps);
    end
end
toc;

%% ================== 7. MDS 位置 RAMSE ==================
[mds_pos_est_hist, RAMSE_pos_history] = ...
    mds_localization_RAMSE(history_dij_est, pos_hist, 1);



%% ================== 8. 作图 ==================
set(0,'defaultAxesFontName','Microsoft YaHei');

figure('Position',[100 100 1200 820]);
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
sgtitle('仿真结果（无预测 + 两步延迟控制）','FontSize',16,'FontWeight','bold');

% offset RAMSE
nexttile;
plot(T_seq, RAMSE_offset_history,'LineWidth',1.8);
title("时钟 offset RAMSE"); grid on; xlabel('时间 (s)');

% skew RAMSE
nexttile;
plot(T_seq, RAMSE_skew_history,'LineWidth',1.8);
title("时钟 skew RAMSE"); grid on; xlabel('时间 (s)');

% 距离 RAMSE
nexttile;
plot(T_seq, RAMSE_relativedij_history, 'LineWidth', 1.8);
title("相对距离 d_{ij} RAMSE"); grid on; xlabel('时间 (s)');

% MDS 位置 RAMSE
nexttile;
plot(T_seq, RAMSE_pos_history,'LineWidth',1.8);
title("MDS 位置 RAMSE (位置误差)"); grid on; xlabel('时间 (s)');

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
plot(T_seq, offset_true_node, 'r', 'LineWidth', 1.3); hold on;
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




