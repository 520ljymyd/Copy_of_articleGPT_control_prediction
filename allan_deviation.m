% function [tau, adev] = allan_deviation(offset, dt, m_vec)
% % 基于“时间偏差序列 offset”计算 Allan 偏差（重叠 Allan 方差）
% %
% % offset : 1xN 或 Nx1 的时钟偏差序列（单位：秒）
% % dt     : 采样间隔（单位：秒）
% % m_vec  : 要计算的 m（整数）列表，对应 tau = m*dt
% %
% % tau    : 实际使用的 tau 列表（秒）
% % adev   : Allan deviation σ_y(tau)
% 
% offset = offset(:);          % 转成列向量
% N = length(offset);
% 
% tau  = [];
% adev = [];
% 
% for k = 1:length(m_vec)
%     m = m_vec(k);
% 
%     % 至少要保证有 (N - 2m) 个二阶差分样本
%     if 2*m >= N
%         break;  % 后面的 m 太大，直接跳出
%     end
% 
%     % 根据标准公式计算系数
%     coeff = 1 / ( 2 * (m^2) * (dt^2) * (N - 2*m) );
% 
%     s = 0;
%     for i = 1:(N - 2*m)
%         d2 = offset(i + 2*m) - 2*offset(i + m) + offset(i);
%         s  = s + d2^2;
%     end
% 
%     tau(end+1,1)  = m * dt;          %#ok<AGROW>
%     adev(end+1,1) = sqrt(coeff * s); %#ok<AGROW>
% end
% 
% end
function [tau_vec, adev_vec] = allan_deviation(offset, dt, L_vec)
% 基于论文中式 (4)(5)(6)，利用 offset 序列计算 Allan 偏差（频率域）
%
% offset : 1xN 或 Nx1 的时钟 offset 序列 (单位：秒)
% dt     : 采样间隔 Δ (单位：秒)
% l_vec  : 想要的平均长度 l 的集合（正整数向量）
% 
% tau_vec  : 对应的 τ = l*dt (秒)
% adev_vec : Allan deviation σ_a(τ)

    % 确保列向量
    offset = offset(:);
    N = length(offset);

    % 1) 由 offset 得到瞬时频率 f(n) —— 对应式 (6)
    f = diff(offset) / dt;    % 长度 N-1
    Nf = length(f);

    % 输出初始化
    tau_vec  = [];
    adev_vec = [];

    for k = 1:length(L_vec)
        L = L_vec(k);

        % 需要至少满足 2l <= Nf，否则无法形成 f^(l)(n) 和 f^(l)(n-l)
        if 2*L > Nf
            % 超出范围直接跳过
            continue;
        end

        % 2) 计算 f^(l)(n) —— 对应式 (5)
        h   = ones(L,1)/L;      % 等权平均滤波器
        f_l = filter(h, 1, f);  % f_l(n) ≈ f^(l)(n)

        % 3) 按式 (4) 求 Allan 方差
        idx = (2*L):Nf;         % n 从 2l 到 Nf
        dif = f_l(idx) - f_l(idx - L);  % f^(l)(n) - f^(l)(n-l)

        allan_var = 0.5 * mean(dif.^2);  
        allan_dev = sqrt(allan_var);

        tau_vec(end+1,1)  = L * dt;       
        adev_vec(end+1,1) = allan_dev;   
    end

end
