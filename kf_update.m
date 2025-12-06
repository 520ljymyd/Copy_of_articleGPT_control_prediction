function [Xk, Hk, Kgain] = kf_update(Xkk_1, Hkk_1, R, Z_n, C_n)
    % 更新
    S     = C_n * Hkk_1 * C_n' + R;     % 观测协方差
    Kgain = Hkk_1 * C_n' / S;           % 卡尔曼增益


    innovation = Z_n - C_n * Xkk_1;     % 观测残差
    Xk = Xkk_1 + Kgain * innovation;    % 状态更新

    I  = eye(size(Hkk_1));
    Hk = (I - Kgain * C_n) * Hkk_1;     % 协方差更新
end