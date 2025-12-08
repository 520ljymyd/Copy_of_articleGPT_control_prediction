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
    %N = length(offset);

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

        tau_vec(end+1,1)  = L * dt;       %#ok<AGROW>
        adev_vec(end+1,1) = allan_dev;    %#ok<AGROW>
    end

end
