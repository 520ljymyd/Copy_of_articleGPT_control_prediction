function [tau1, tau2] = calcLoopCoef(BW, zeta, k)
% 计算二阶环路(PLL/FLL)的时间常数.
%
% 输入:
%   BW  - 带宽 (Hz)
%   zeta - 阻尼系数
%   k    - “等效鉴别器增益 × NCO增益” (loop gain)
%
% 输出:
%   tau1, tau2 - 连续时间模型中的时间常数

% 自然角频率 (rad/s)
Wn = BW * 8 * zeta / (4 * zeta^2 + 1);

% 时间常数
tau1 = k / (Wn^2);
tau2 = 2.0 * zeta / Wn;
end
