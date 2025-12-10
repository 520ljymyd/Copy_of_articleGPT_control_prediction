function [mu_omega, state] = mu_omega_update(delta_hat, omega_hat, state, param)
% 实现论文中 μ_ω(n+2) = g(δ̂(n+2), ω̂(n+2))
%
% 输入:
%   delta_hat  - clock offset 估计 (秒)
%   omega_hat  - clock skew 估计
%   state      - 环路内部状态 (结构体, fields: s0, s1)
%   param      - 参数结构体(见 main 中的设置)
%
% 输出:
%   mu_omega   - 频率控制量 (作为 Λ 中相应分量)
%   state      - 更新后的状态

% ---------- 0. 取出必要参数 ----------
Tc        = param.Tc;

tau0_PLL  = param.tau0_PLL;
tau1_PLL  = param.tau1_PLL;
tau2_PLL  = param.tau2_PLL;
tau1_FLL  = param.tau1_FLL;
tau2_FLL  = param.tau2_FLL;

Kp_delta  = param.Kp_delta;
Kp_omega  = param.Kp_omega;
mu_max    = param.omega_th;  % 频率控制量限幅

% ---------- 1. 构造“相位误差”和“频率误差” ----------
% 这里直接把 δ̂, ω̂ 当作误差源, 再乘比例系数
phaseErr  = Kp_delta * delta_hat;
freqErr   = Kp_omega * omega_hat;

% ---------- 2. FLL + PLL 组合环路滤波 ----------
% 2.1 更新第一个积分状态 s0 (主要由 FLL 驱动, 同时含少量相位项)
s0_new = state.s0 + (tau0_PLL * phaseErr + tau1_FLL * freqErr) * Tc;

% 2.2 更新第二个积分状态 s1 (第三阶 PLL + 第二阶 FLL)
s1_new = state.s1 + (tau1_PLL * phaseErr + s0_new + tau2_FLL * freqErr) * Tc;

% 2.3 输出频率控制量 (三阶 PLL 输出 + s1_new)
mu_omega_raw = s1_new + tau2_PLL * phaseErr;

% ---------- 3. 幅度限幅 ----------
if mu_omega_raw > mu_max
    mu_omega = mu_max;
elseif mu_omega_raw < -mu_max
    mu_omega = -mu_max;
else
    mu_omega = mu_omega_raw;
end

% ---------- 4. 更新状态并返回 ----------
state.s0 = s0_new;
state.s1 = s1_new;

end
