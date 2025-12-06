function [rhomat_k, dij_true_mat] = pseudorange_STWR( ...
    N, pos_init, vel, offset_true_k,skew_true_k, T_n, c, sigma_t)
    %输入 ：
    % N               节点数
    % pos_init        3xN 初始位置
    % vel             3xN 速度
    % offset_true_k   Nx1 真实时钟偏差
    % skew_true_k     Nx1 真实时钟频偏
    % T_n            当前全局时间
    % c               光速
    % sigma_t        伪距测量噪声标准差（秒）
    %输出：
    % rhomat_k       2ψx1 伪距测量值向量（米）
    % dij_true_mat   NxN 真实几何距离矩阵（米）

psi = N * (N-1) / 2;
rhomat_k = zeros(2*psi,1);
dij_true_mat = zeros(N,N);

idx = 1;

for i = 1:N-1
    for j = i+1:N
        %% ===== 1. 真实世界的全局时间 =====
        % UAV 的真实运动只跟 T_n 演化，与 offset 无关


        pi = pos_init(:,i) + vel(:,i) * T_n * (1 + skew_true_k(i));
        pj = pos_init(:,j) + vel(:,j) * T_n * (1 + skew_true_k(j));

        %% ===== 2. 真实几何距离 =====
        d_ij = norm(pj - pi);
        dij_true_mat(i,j) = d_ij;
        dij_true_mat(j,i) = d_ij;

        %% ===== 3. 传播时间 =====
        chi = d_ij / c;

        %% ===== 4. clock offset 差 =====
        delta_ij = offset_true_k(j) - offset_true_k(i);

        %% ===== 5. measurement noise 观测噪声 =====
        alpha_ij = sigma_t * randn();
        alpha_ji = sigma_t * randn();

        %% ===== 6. 观测量=====
        z_ij = chi - delta_ij + alpha_ij;
        z_ji = chi + delta_ij + alpha_ji;

        %% ===== 7. convert to meters 转换为距离=====
        rhomat_k(idx)   =c * z_ij;
        rhomat_k(idx+1) =c * z_ji;

        idx = idx + 2;
    end
end







% function [rhomat_k, dij_true_mat] = pseudorange_STWR( ...
%     N, pos_hist_k, vel_hist_k, offset_true_k, skew_true_k, ...
%     T_n, c, sigma_t)
% 
% psi = N * (N-1) / 2;
% rhomat_k = zeros(2*psi,1);
% dij_true_mat = zeros(N,N);
% 
% idx = 1;
% 
% % ======== STWR 的关键控制参数（论文中的“非理想因素”）===========
% tau_hw_i = 3e-9;   % 节点 i 硬件延迟 (秒)
% tau_hw_j = 3.5e-9; % 节点 j 硬件延迟，不对称 (秒)
% 
% sigma_tracking = 3e-10;    % 码追踪噪声 (秒)
% sigma_los = 0.1;           % LOS 方向扰动控制量
% sigma_dir = 1e-3;          % 方向小扰动
% 
% for i = 1:N-1
%     for j = i+1:N
% 
%         %% ================================================================
%         %  (1) 真实几何位置 —— 不再用 pos_init + v*T_n，而是用 pos_hist
%         %      这是真实轨迹（包含你后面可能加入的非线性、扰动）
%         % ================================================================
%         pi = pos_hist_k(:,i);
%         pj = pos_hist_k(:,j);
% 
%         %% ========= 微小扰动方向：论文真实情况中 UAV 不会完美直线 ===========
%         pj = pj + sigma_dir * randn(3,1);
%         pi = pi + sigma_dir * randn(3,1);
% 
%         %% ================================================================
%         % (2) 真实几何距离（正确的真值）
%         % ================================================================
%         d_ij = norm(pj - pi);
%         dij_true_mat(i,j) = d_ij;
%         dij_true_mat(j,i) = d_ij;
% 
%         %% ================================================================
%         % (3) 真实传播时间（无近似）
%         % ================================================================
%         chi_true = d_ij / c;
% 
%         %% ================================================================
%         % (4) STWR **异步传输时间**
%         %     论文核心：Gi(n) ≠ Gj(n)，造成 forward/backward 不对称
%         % ================================================================
%         Gi_n = T_n + offset_true_k(i);
%         Gj_n = T_n + offset_true_k(j);
% 
%         % forward link
%         t_tx_ij = Gi_n;
%         % reverse link
%         t_tx_ji = Gj_n;
% 
%         %% ================================================================
%         % (5) 相对运动造成的“非完美 χji ≈ χij”近似误差（论文重点）
%         % ================================================================
%         %
%         %   χji ≈ χij(Gj + χji)
%         %
%         %   我们人为加入一个小偏移，模拟论文中的该非理想项
%         %
%         chi_ij = chi_true;
%         chi_ji = chi_true + (vel_hist_k(:,j)' * (pj - pi)) / (c^2);
% 
%         %% ================================================================
%         % (6) 非对称硬件延迟（论文真实存在）
%         % ================================================================
%         chi_ij = chi_ij + tau_hw_i;
%         chi_ji = chi_ji + tau_hw_j;
% 
%         %% ================================================================
%         % (7) 码跟踪噪声（tracking loop 限制）
%         % ================================================================
%         chi_ij = chi_ij + sigma_tracking * randn();
%         chi_ji = chi_ji + sigma_tracking * randn();
% 
%         %% ================================================================
%         % (8) clock offset 的影响（论文式 (8)-(9)）
%         % ================================================================
%         delta_ij = offset_true_k(j) - offset_true_k(i);
% 
%         %% ================================================================
%         % (9) 最终伪距观测（论文的 STWR 测量方程）
%         % ================================================================
%         z_ij = chi_ij - delta_ij + sigma_t * randn();
%         z_ji = chi_ji + delta_ij + sigma_t * randn();
% 
%         %% ================================================================
%         % (10) 输出伪距（米）
%         % ================================================================
%         rhomat_k(idx)   = c * z_ij;
%         rhomat_k(idx+1) = c * z_ji;
% 
%         idx = idx + 2;
%     end
% end
