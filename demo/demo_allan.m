%% 从 offset 历史计算 Allan 偏差，并与理论曲线对比

offset_hist = history_offset_true(2,:);     % 这里填入 offset 序列 (秒)，1xN 或 Nx1

N = numel(offset_hist);

% 选取一组 m，使 tau 在 [dt, N*dt/10] 左右的 log 区间内
m_max  = floor(N/4);                        % 确保 2*m < N
m_vec  = unique(round(logspace(0, log10(m_max), 20)));  

% 1) 用 offset 序列计算 Allan 偏差
[tau_emp, adev_emp] = allan_deviation(offset_hist, dt, m_vec);

% 2) 计算理论自由运行 Allan 偏差曲线进行对比

% q1 = 1e-3;
% q2 = 1e-9;

allan_free_running = sqrt(q1 ./ tau_emp + q2 .* tau_emp / 3);  % 对应论文式 (7) 的 σ_a(τ)

% 3) 画图
figure;
loglog(tau_emp, adev_emp, 'bo-', 'LineWidth', 1.6, 'MarkerSize', 6);
hold on;
loglog(tau_emp, allan_free_running, 'r>-', 'LineWidth', 1.6, 'MarkerSize', 6);

grid on; grid minor;
xlabel('\tau (sec)');
ylabel('Allan deviation');
legend('clock steering non-prediction', 'Theoretical WFM+RWFM free running', 'Location', 'SouthWest');
title('Clock Allan deviation: empirical vs theoretical');

%限定 x, y 轴范围
% xlim([1e0, 1e5]);
% ylim([1e-16, 1e-10]);
