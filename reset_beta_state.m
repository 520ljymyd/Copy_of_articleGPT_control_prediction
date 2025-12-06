function [Xk, Hk] = reset_beta_state(Xk, Hk, phi, sigma_beta0)

    % 不要清零 beta —— 保留其当前值
    % Xk(1:phi) = Xk(1:phi);  

    % 重置 beta 的协方差为较大值
    Hk(1:phi,1:phi) = (sigma_beta0^2)*eye(phi);

    % 清空 cross-covariance
    % Hk(1:phi,phi+1:end) = 0;
    % Hk(phi+1:end,1:phi) = 0;

end
