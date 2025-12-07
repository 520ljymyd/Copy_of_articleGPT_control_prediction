clc; clear; close all; format long;

%% ===============================
%  实验目的：
%  在 “无时钟误差” 的情况下，用最小二乘拟合 beta，
%  观察相对距离 RAMSE 随时间是否发散。
% ================================

%% 参数设置
c = 299792458;           % 光速
T_end = 600;              % 总时间
dt    = 0.02;
T_seq = 0:dt:T_end;
Nsteps = length(T_seq);

K = 3;                   % 多项式阶数（可改成 1 比较）
sigma_t = 1e-7;          % 测量噪声 (秒)

%% 设置两架无人机的真实初始位置与速度（简单匀速）
p0_i = [ 0; 0; 0 ];
v_i  = [10;  2;  0];     % m/s

p0_j = [10000; 3000; 0];
v_j  = [-3; 8; 0];       % m/s

%% ============== 1. 生成真实传播时延 χ_true(t) = d(t)/c ============
chi_true = zeros(1,Nsteps);

for k = 1:Nsteps
    t = T_seq(k);
    pi = p0_i + v_i * t;
    pj = p0_j + v_j * t;
    d_ij = norm(pj - pi);     % 真实距离
    chi_true(k) = d_ij / c;   % 秒
end

%% 在 χ_true 上加入噪声模拟 z = χ + 噪声
z_meas = chi_true + sigma_t * randn(size(chi_true));

%% ============ 2. 用最小二乘拟合 beta =====================
% 构造回归矩阵 T = [1, t, t^2]
Phi = zeros(Nsteps, K+1);
for k = 1:Nsteps
    Phi(k,:) = T_seq(k).^(0:K);
end

% LS 解：beta_hat = (Phi'Phi)^(-1) Phi' z
beta_hat = (Phi' * Phi) \ (Phi' * z_meas.');

%% =========== 3. 用拟合的 beta_hat 重构 chi(t)，再算距离 ===========
chi_hat = Phi * beta_hat;
d_hat   = c * chi_hat;        % 估计距离
d_true  = c * chi_true;       % 真实距离

%% =========== 4. 计算 RMSE 随时间的变化 ======================
RMSE_dist = zeros(1,Nsteps);
for k = 1:Nsteps
    RMSE_dist(k) = sqrt( mean((d_hat(k)-d_true(k))^2) );
end

%% =========== 5. 画图 ======================
figure; plot(T_seq, RMSE_dist, 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Distance RMSE (m)');
title(sprintf('Distance RMSE, K=%d, no clock error', K));
