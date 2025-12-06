function [position_rel_k,velocity_rel_k] = pv_update(position_rel_k_1, velocity_rel_k_1, accelerate_rel_k_1, dt, sigma_a)
% POSITION_VELOCITY 根据简单的运动模型更新无人机的位置和速度
%
% 输入:
%   CurrentPosition : 3xN 矩阵，当前位置 [x; y; z]
%   CurrentVelocity : 3xN 矩阵，当前速度 [vx; vy; vz]
%   CurrentAccelerate : 3xN 矩阵，当前加速度 [ax; ay; az]
%   Delta           : 仿真时间步长 (秒)
%   sigma_a         : 加速度过程噪声的标准差 (m/s^2)
%
% 输出:
%   NextPosition    : 3xN 矩阵，下一时刻的位置
%   NextVelocity    : 3xN 矩阵，下一时刻的速度

%% 检查输入维度是否一致
Np = size(position_rel_k_1, 2);
Nv = size(velocity_rel_k_1, 2);
if Np ~= Nv
    error('Position and Velocity matrices must have the same number of columns (number of UAVs).');
end

%%---------- 1. 引入加速度过程噪声 ----------
% 生成 3xN 的零均值高斯白噪声，代表随机加速度扰动
% 维度：3 (x,y,z) x N (UAVs)
acceleration_noise = sigma_a * randn(3, Np); 

%% 实际加速度 = 标称加速度 + 噪声
effective_acceleration = accelerate_rel_k_1 + acceleration_noise;

%% 2. 位置更新 (p_next = p_current + v_current * Delta)
% 保持与您原始的简化 CV/CA 模型一致
position_rel_k = position_rel_k_1 + dt * velocity_rel_k_1;

%% 3. 速度更新 (v_next = v_current + a_effective * Delta)
velocity_rel_k = velocity_rel_k_1 + dt * effective_acceleration;

end