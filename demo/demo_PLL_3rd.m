clear; clc; close all; format long;

%% ================== PLL 参数设置 ==================
PLL_band = 0.35;                            % PLL 带宽
FLL_band = 0.06;                            % FLL 带宽
zeta = 1/sqrt(2);                           % 阻尼比
sigma_osc = 1e-11;                          % 示例 VCXO 短期稳定度
mu_omega_max = sqrt(1.05^2 - 1) * sigma_osc;

offset_init = 0.004;                      % 初始时钟偏移 (秒)
skew_init = 1e-6;                        % 初始时钟频偏 (ppm)

q_WFM = 1e-10;                           % WFM 过程噪声强度 
q_RWFM = 1e-16;                          % RWFM 过程噪声强度

dt = 0.02;                                 % 时间步长 (秒)
Nsteps = 1500;                             % 总时间步数


offset_k = offset_init;          % 初始时钟偏移
skew_k   = skew_init;            % 初始时钟频偏
x = [offset_k; skew_k];            % 初始状态向量 [offset; skew]
F = [1, dt; 0, 1];               % 状态转移矩阵
offset_hist = [];               % 记录偏移历史
skew_hist = [];                 % 记录频偏历史

for k = 1:Nsteps
    % 记录历史
    offset_hist(k) = offset_k; %#ok<SAGROW>
    skew_hist(k)   = skew_k;   %#ok<SAGROW>

    X = F * x;                    % 状态预测
    offset_k = X(1);
    skew_k   = X(2);    
    x = X;

    
end
