function [offset_rel_k, skew_rel_k] = offset_skew_update(offset_rel_k_1, skew_rel_k_1, ...
                                                          dt, N, q1, q2)
% ================================================================
% 功能：
%   根据时钟状态空间模型，更新各节点的 offset 和 skew
%   模型形式：
%       [δ_i(n+1)]   [1 Δ] [δ_i(n)] + [ε_δ,i(n)]
%       [ω_i(n+1)] = [0 1] [ω_i(n)] + [ε_ω,i(n)]
%
% 输入：
%   offset_rel_k_1 : 1×N 向量，上时刻各节点 offset
%   skew_rel_k_1   : 1×N 向量，上时刻各节点 skew
%   dt          : 采样时间间隔
%   N              : 节点数量
%   q1, q2         : 过程噪声参数
%
% 输出：
%   offset_rel_k   : 1×N 向量，本时刻各节点 offset
%   skew_rel_k     : 1×N 向量，本时刻各节点 skew
% ================================================================

% ---------- Step 1. 状态转移矩阵 ----------
F = [1, dt;
     0, 1];

% ---------- Step 2. 噪声协方差矩阵 ----------
Q = [ q1*dt + (q2*(dt^3))/3,  (q2*(dt^2))/2;
        (q2*(dt^2))/2      ,     q2*dt       ];

% ---------- Step 3. 生成 N 个节点的随机噪声 ----------
% mvnrnd(mean, cov, num_samples) → 输出 num_samples×dim
noise_samples = mvnrnd([0, 0], Q, N)';  % (2×N)
epsilon_delta = noise_samples(1, :);    % 1×N
epsilon_omega = noise_samples(2, :);    % 1×N

% ---------- Step 4. 执行状态更新 ----------
% 对每个节点执行: x_i(n+1) = F*x_i(n) + w_i(n)
offset_rel_k = zeros(1, N);
skew_rel_k   = zeros(1, N);

for i = 1:N
    x_prev = [offset_rel_k_1(i); skew_rel_k_1(i)];
    w_i    = [epsilon_delta(i); epsilon_omega(i)];
    x_curr = F * x_prev + w_i;
    offset_rel_k(i) = x_curr(1);
    skew_rel_k(i)   = x_curr(2);
end
offset_rel_k(1) = 0;
skew_rel_k(1) = 0;
end
