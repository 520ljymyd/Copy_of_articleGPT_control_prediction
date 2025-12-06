function [delta_est_k, omega_est_k, history_delta_est, history_omega_est] = ...
    store_clock_est(N, Xk, phi, k, history_delta_est, history_omega_est)

    Xk = Xk(:);

    delta_est_k = zeros(N,1);
    omega_est_k = zeros(N,1);

    % 参考节点 1 固定为 0
    delta_est_k(1) = 0;
    omega_est_k(1) = 0;

    idx_delta_start = phi + 1;
    idx_delta_end   = phi + (N-1);
    idx_omega_start = idx_delta_end + 1;
    idx_omega_end   = idx_omega_start + (N-2);

    delta_est_k(2:N) = Xk(idx_delta_start:idx_delta_end);
    omega_est_k(2:N) = Xk(idx_omega_start:idx_omega_end);

    history_delta_est(:,k) = delta_est_k;
    history_omega_est(:,k) = omega_est_k;
end
