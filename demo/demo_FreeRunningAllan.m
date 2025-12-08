% clear; clc; close all; format long;

% %% ================== 模型参数 ================== 
% dt = 0.02; % 采样时间
% Nsteps = 5e5; % 增加点数以获得更平滑的 Allan 曲线
% offset_init = 0.003; 
% skew_init = 1e-10; 

% % 噪声强度参数 (根据你的设置)
% % q1: White FM (对应的相位扩散)
% % q2: Random Walk FM (对应的频率扩散)
% q1 = 1e-20; 
% q2 = 1e-24; 

% %% ================== 状态空间矩阵 ================== 
% F = [1, dt; 0, 1];

% % 离散化过程噪声协方差矩阵 Q (Van Loan方法近似或解析解)
% Q = [ q1*dt + (q2*(dt^3))/3,  (q2*(dt^2))/2;
%       (q2*(dt^2))/2       ,     q2*dt        ];

% %% ================== 状态初始化 ================== 
% x = [offset_init; skew_init]; 
% offset_hist = zeros(1,Nsteps);
% offset_hist(1) = x(1); 

% %% ================== 噪声生成 (关键修正) ================== 
% % 使用 mvnrnd 根据协方差矩阵 Q 生成噪声
% % Q 必须是半正定矩阵
% rng(100); % 固定随机种子以便复现
% W = mvnrnd([0, 0], Q, Nsteps)'; % 结果为 2xNsteps

% %% ================== 主循环 ================== 
% for k = 2:Nsteps 
%     % ---- 状态自由传播 ---- 
%     % 使用预生成的符合 Q 分布的噪声
%     w_k = W(:, k-1);
    
%     X_k = F*x + w_k; 
    
%     % ---- 更新状态 ---- 
%     offset_hist(k) = X_k(1); 
%     x = X_k; 
% end

% %% ================== Allan 方差分析 ================== 
% % 选取 tau 序列
% N = length(offset_hist);
% m_max  = floor(N/4); 
% % 使得 tau 在 log 轴上均匀分布
% m_vec  = unique(round(logspace(0, log10(m_max), 50)));  

% % 1) 计算仿真数据的 Allan 偏差
% [tau_emp, adev_emp] = allan_deviation(offset_hist, dt, m_vec);

% % 2) 计算理论曲线
% % 理论公式: sigma^2 = q1/tau + q2*tau/3
% allan_free_running = sqrt(q1 ./ tau_emp + q2 .* tau_emp / 3); 

% %% ================== 画图 ================== 
% figure;
% loglog(tau_emp, adev_emp, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 10);
% hold on;
% loglog(tau_emp, allan_free_running, 'r--', 'LineWidth', 2);

% grid on; grid minor;
% xlabel('\tau (sec)');
% ylabel('Allan Deviation \sigma_y(\tau)');
% legend('Simulated Free Running', 'Theoretical Prediction', 'Location', 'Best');
% title(['Clock Free Running Analysis (q1=' num2str(q1) ', q2=' num2str(q2) ')']);

% % --------------------------------------------------------
% % 子函数: Allan 偏差计算
% % --------------------------------------------------------
% function [tau_vec, adev_vec] = allan_deviation(offset, dt, L_vec)
%     offset = offset(:);
    
%     % 1) 计算平均频率 (y_k)
%     % 注意: diff(x)/dt 得到的是频率数据
%     f = diff(offset) / dt; 
%     Nf = length(f);

%     tau_vec  = zeros(length(L_vec), 1);
%     adev_vec = zeros(length(L_vec), 1);

%     for k = 1:length(L_vec)
%         L = L_vec(k);
%         if 2*L > Nf
%             continue;
%         end

%         % 2) 构造均值滤波器计算平均频率 y_bar
%         h = ones(L,1)/L;
        
%         % filter 会产生延迟，前 L-1 个点是过渡态
%         f_l = filter(h, 1, f); 
        
%         % 3) Allan 方差计算
%         % 只有从索引 2*L 开始的数据是有效的完全重叠差分
%         % f_l(i) 是 y_bar_k, f_l(i-L) 是 y_bar_{k-1}
%         idx = (2*L):Nf; 
%         dif = f_l(idx) - f_l(idx - L); 

%         allan_var = 0.5 * mean(dif.^2);  
%         adev_vec(k) = sqrt(allan_var);
%         tau_vec(k)  = L * dt; 
%     end
    
%     % 移除未计算的零值
%     mask = tau_vec > 0;
%     tau_vec = tau_vec(mask);
%     adev_vec = adev_vec(mask);
% end
clear; clc; close all; format long;

%% ================== 模型参数 ================== 
dt      = 0.02;      % 采样时间
Nsteps  = 5e5;       % 仿真步数
offset_init = 0.0024; 
skew_init   = 1e-10; 

% 噪声强度参数
% q1: White FM
% q2: Random Walk FM
q1 = 1e-12; 
q2 = 1e-16; 

%% ================== 状态空间矩阵 ================== 
F = [1, dt;
     0, 1];

Q = [ q1*dt + (q2*(dt^3))/3,  (q2*(dt^2))/2;
      (q2*(dt^2))/2        ,     q2*dt       ];

%% ================== 状态初始化 ==================  
x = [offset_init; skew_init]; 

offset_hist = zeros(Nsteps,1);
offset_hist(1) = x(1);

%% ================== 噪声生成 ================== 
% 生成 Nsteps-1 个增量噪声（对应 2~Nsteps 这 Nsteps-1 步）
rng(100); 
W = mvnrnd([0, 0], Q, Nsteps-1)';   % 2 × (Nsteps-1)

%% ================== 主循环 ================== 
for k = 2:Nsteps 
    w_k = W(:, k-1);      % 第 k-1 个增量噪声
    x   = F * x + w_k;    % 状态更新
    offset_hist(k) = x(1);
end

%% ================== Allan 方差分析 ================== 
% 用 offset 序列计算 Allan 偏差
N      = length(offset_hist);
m_max  = floor(N/4); 
m_vec  = unique(round(logspace(0, log10(m_max), 50)));  

[tau_emp, adev_emp] = allan_deviation(offset_hist, dt, m_vec);

% 理论 Allan 曲线：与 q1、q2 一致
allan_free_running = sqrt(q1 ./ tau_emp + q2 .* tau_emp / 3); 

%% ================== 画图 ================== 
figure;
loglog(tau_emp, adev_emp, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 10); hold on;
loglog(tau_emp, allan_free_running, 'r--', 'LineWidth', 2);

grid on; grid minor;
xlabel('\tau (sec)');
ylabel('Allan Deviation \sigma_y(\tau)');
legend('Simulated Free Running', 'Theoretical Prediction', 'Location', 'Best');
title(['Clock Free Running Analysis (q1=' num2str(q1) ', q2=' num2str(q2) ')']);



% --------------------------------------------------------
% 子函数: Allan 偏差计算
% --------------------------------------------------------
% function [tau_vec, adev_vec] = allan_deviation(offset, dt, L_vec)
%     offset = offset(:);
    
%     % 由 offset 得到频率序列
%     f  = diff(offset) / dt; 
%     Nf = length(f);

%     tau_vec  = zeros(length(L_vec), 1);
%     adev_vec = zeros(length(L_vec), 1);

%     for k = 1:length(L_vec)
%         L = L_vec(k);
%         if 2*L > Nf
%             continue;
%         end

%         % 长度为 L 的滑动平均
%         h   = ones(L,1)/L;
%         f_l = filter(h, 1, f); 

%         % 从 2L 开始做重叠差分
%         idx = (2*L):Nf; 
%         dif = f_l(idx) - f_l(idx - L); 

%         allan_var   = 0.5 * mean(dif.^2);  
%         adev_vec(k) = sqrt(allan_var);
%         tau_vec(k)  = L * dt; 
%     end
    
%     mask    = tau_vec > 0;
%     tau_vec = tau_vec(mask);
%     adev_vec = adev_vec(mask);
% end
