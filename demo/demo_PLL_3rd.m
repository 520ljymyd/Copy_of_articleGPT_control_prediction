clear; clc; close all; format long;

%% ================== 模型参数 ==================
dt = 0.02;          
Nsteps = 22200;

offset_init = 0.004;
skew_init   = 1e-6;

% 合理噪声强度（否则PLL不可能收敛）
q_WFM  = 1e-12;
q_RWFM = 1e-16;

%% ================== PLL 参数 ==================
PLL_band = 0.35;
zeta = 1/sqrt(2);

wn = 2*pi*PLL_band;

Kp = 2*zeta*wn*dt;
Ki = (wn*dt)^2;

fprintf("二阶 PLL 系数:  Kp = %.3e,  Ki = %.3e\n", Kp, Ki);

%% ================== 状态初始化 ==================
x = [offset_init; skew_init];
F = [1 dt; 0 1];

z_int = 0;     % PLL 积分器

offset_hist = zeros(1,Nsteps);
skew_hist   = zeros(1,Nsteps);

offset_hist(1) = x(1);
skew_hist(1)   = x(2);

%% ================== 主循环 ==================
for k = 2:Nsteps

    % ---- 状态自由传播 ----
    w = [q_WFM*randn(); 
         q_RWFM*randn()];
    X = F*x + w;

    % ---- PLL 控制误差（使用 offset） ----
    e = -X(1);
    X(1) = X(1) + e;   % PLL 锁定后，offset 误差清零
    % ---- 正确的二阶 PLL 更新 ----
    z_int = z_int + Ki * e;     % 积分器必须保留状态
    uk = Kp * e + z_int;        % 控制输入

    % ---- 控制作用于 skew ----
    X(2) = X(2) + uk;

    % ---- 保存结果并更新状态 ----
    offset_hist(k) = X(1);
    skew_hist(k)   = X(2);
    x = X;
end

%% ================== 绘图 ==================
figure;
subplot(2,1,1); plot(offset_hist, 'LineWidth', 1.2); grid on;
title("Offset (秒)");

subplot(2,1,2); plot(skew_hist, 'LineWidth', 1.2); grid on;
title("Skew 频偏");
