function [history_dij_est, RAMSE_relativedij_history] = ...
    update_distance_and_RAMSE(N, K, c, T_n, Xk, k, ...
    history_dij_est, history_dij_true, ...
    RAMSE_relativedij_history)

Xk = Xk(:);
psi = N*(N-1)/2;
phi = (K+1)*psi;

beta_hat = Xk(1:phi);
t_vec = (T_n.^(0:K)).';

col = 1;
for i=1:N-1
    for j=i+1:N
        b_ij = beta_hat(col:col+K);
        chi_hat = b_ij' * t_vec;
        d_hat   = c * chi_hat;
        history_dij_est(i,j,k) = d_hat;
        history_dij_est(j,i,k) = d_hat;
        col = col + (K+1);
    end
end

% 上三角向量
d_true_vec = [];
d_est_vec  = [];

for i=1:N-1
    for j=i+1:N
        d_true_vec(end+1,1) = history_dij_true(i,j,k);
        d_est_vec(end+1,1)  = history_dij_est(i,j,k);
    end
end

dist_err_vec = d_est_vec - d_true_vec;
dist_err_vec = dist_err_vec-mean(dist_err_vec);
% 正确的 RMSE
RAMSE_relativedij_history(k) = sqrt( mean(dist_err_vec.^2) );