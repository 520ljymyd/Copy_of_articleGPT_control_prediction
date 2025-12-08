%%  some problem in demo_adjustment.m ignore this file
clear; clc; close all; format long;

%% ================== 模型参数 ================== 
dt = 0.02;
Nsteps = 1e5;
offset_init = 0.004; 
skew_init = 0.8e-8; % 合理噪声强度
% 噪声强度参数
q1 = 1e-13;
q2 = 1e-19; 
%% ================== 控制参数 ==================
delta_th = 5e-7; % offset 操作阈值 
omega_th = 3.2e-12; % skew 操作限幅 
H_max = 2; % 计数器大小

%% ================== PLL 参数 ================== 
PLL_band = 0.05; % PLL 带宽 
FLL_band = 0.06; % FLL 带宽
zeta = 1/sqrt(2); 
wn = 2*pi*PLL_band; % 定义环路自然频率 



z_int = 0; % PLL 积分器 
Kp = 2*zeta*wn*dt; 
Ki = (wn*dt)^2;

%% ================== 状态初始化 ================== 
x = [offset_init; skew_init]; 
F = [1, dt; 0, 1];


offset_hist = zeros(1,Nsteps);
skew_hist = zeros(1,Nsteps); 
offset_hist(1) = x(1); 
skew_hist(1) = x(2); 

% ---- 恢复初始状态定义 ----
z1 = 0;
z2 = 0; 
z3 = 0; 
y1 = 0;
y2 = 0; 



K1 = 3 * wn * dt;
K2 = 3 * (wn*dt)^2;
K3 = (wn*dt)^3;
G1 = 2 * FLL_band * dt;
G2 = (FLL_band * dt)^2;

Q = [ q1*dt + (q2*(dt^3))/3,  (q2*(dt^2))/2;
        (q2*(dt^2))/2      ,     q2*dt       ];


%% ================== 主循环 ================== 
for k = 2:Nsteps 
    % ---- 状态自由传播 ---- 
    W = mvnrnd([0, 0], Q, Nsteps-1)';   % 2 × (Nsteps-1)
    w_k = W(:, k-1);      % 第 k-1 个增量噪声
    X_k = F*x + w_k; 
    
    % ---- PLL 控制误差（使用 offset） ---- 
    % e1 = OFFSET_CONTROL(X_k(1), delta_th, H_max); 
    %e1 = -0.1 * X(1); % 直接使用 offset 作为误差
    % X_k(1) = X_k(1) + e1; % offset 控制作用于 offset 
    
    % ---- 三阶 PLL 更新 (使用 OMEGA_CONTROL 函数) ---- 
    % [u1, z1, z2, z3, y1, y2] = OMEGA_CONTROL(e1, X_k(2), z1, z2, z3, y1, y2, K1, K2, K3, G1, G2, omega_th); 
    
    %-------- 二阶 PLL 更新 ----
    % z_int = z_int + Ki * e1; % 积分器必须保留状态 
    % u1 = Kp * e1 + z_int; % 控制输入
    
    % ---- 控制作用于 skew ---- 
    % X_k(2) = X_k(2) + u1; 
    
    % ---- 保存结果并更新状态 ---- 
    offset_hist(k) = X_k(1); 
    skew_hist(k)   = X_k(2); 
    x = X_k; 
end 

%% ================== 绘图 ================== 
figure; 
subplot(2,1,1); 
plot(offset_hist, 'LineWidth', 1.2); 
grid on; 
title("Offset (秒)"); 

subplot(2,1,2); 
plot(skew_hist, 'LineWidth', 1.2); 
grid on; 
title("Skew 频偏"); 


% %% ================== 函数定义 ==================
% function [muOMEGA, z1, z2, z3, y1, y2] = OMEGA_CONTROL(offsetest_control, skewest, z1, z2, z3, y1, y2, Kp1, Kp2, Kp3, G1, G2, mu_max) 
%     % --- PLL（三阶） --- 
%     e_phi = offsetest_control; 
%     z1 = z1 + Kp1 * e_phi; 
%     z2 = z2 + Kp2 * e_phi; 
%     z3 = z3 + Kp3 * e_phi; 
%     u_PLL = z1 + z2 + z3; 

%     % --- FLL（二阶） --- 
%     e_f = skewest; 
%     y1 = y1 + G1 * e_f; 
%     y2 = y2 + G2 * e_f;   
%     u_FLL = y1 + y2; 


%     % --- g函数控制 --- 
%     muOMEGA = u_PLL + u_FLL; 

%     if muOMEGA > mu_max || muOMEGA == mu_max 
%         muOMEGA = mu_max; 
%     else 
%         muOMEGA = muOMEGA ; %#ok<ASGSL>
%     end 
% end 



% function [muOFFSET] = OFFSET_CONTROL(offsetest, offset_threshold, H_max) 
%     if abs(offsetest) > offset_threshold || abs(offsetest) == offset_threshold 
%         muOFFSET = -offsetest * (H_max / (2*pi)); 
%     else 
%         muOFFSET = 0; 
%     end 
% end