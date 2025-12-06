%% Free-running clocks with both WFM and RWFM (have a turn-up)

clear; clc; close all;

tau = logspace(0,5,20);   % 1 ~ 1e5 s

% 下面这两组 q1,q2 是示例，用来产生“先降后升”的形状
% 根据原文图微调这两个值
% q1_c1 = 1e-22;   % Clock 1: WFM 强度
% q2_c1 = 1.44e-26;   % Clock 1: RWFM 强度


%参考论文Performance Analysis of Kalman-Filter-Based Clock 
% Synchronization in IEEE 1588 Networks  结尾有图

q1_c1     = 1.0e-12;              % clock WFM  (sigma_theta)^2
q2_c1     = 1.2e-13;              % clock RWFM (sigma_gama)^2

q1_c2 = 3e-21;   % Clock 2
q2_c2 = 3e-25;


allan1 = sqrt(q1_c1 ./ tau + q2_c1 .* tau / 3);


allan2 = sqrt(q1_c2 ./ tau + q2_c2 .* tau / 3);

figure;
loglog(tau, allan1, 'b>-', 'LineWidth', 1.6, 'MarkerSize', 6); hold on;
loglog(tau, allan2, 'g^-', 'LineWidth', 1.6, 'MarkerSize', 6);

grid on; grid minor;
xlabel('\tau (sec)');
ylabel('Allan deviation');
legend('Free running (clock 1)','Free running (clock 2)','Location','SouthWest');
title('Free-running clocks with WFM + RWFM (with turning point)');
%限制x,y轴的范围
xlim([1e-2,1e7]);
ylim([1e-20,1e-8]);    
