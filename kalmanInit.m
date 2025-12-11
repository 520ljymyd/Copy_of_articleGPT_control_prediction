function [Lmat, Q, Xk_1, Hk_1, R, Zk, Bmat] = kalmanInit(N, K, dt, q1, q2, sigma_t)

psi = N*(N-1)/2;
phi = (K+1) * psi;
state_dim = phi + 2 * (N-1);

%% 状态转移矩阵 L
Lmat = eye(state_dim);
% δ_k+1 = δ_k + dt * ω_k
Lmat(phi+1 : phi+(N-1), phi+(N-1)+1 : phi+2*(N-1)) = dt * eye(N-1);

%% 初始状态 X(0|0)
Xk_1 = zeros(state_dim, 1);

%% 初始协方差 H(0|0)
Hk_1 = zeros(state_dim);

Hk_1(1:phi, 1:phi) = 1 * eye(phi);              
Hk_1(phi+1:end, phi+1:end) =1 * eye(2*(N-1));    
%% 过程噪声 Q
Q = zeros(state_dim);

Q1 = (q1*dt + q2*(dt^3)/3)     * eye(N-1);
Q2 = (q2*(dt^2)/2)             * eye(N-1);
Q3 = (q2*(dt^2)/2)             * eye(N-1);
Q4 = (q2*dt)                   * eye(N-1);

Q_CLK = [Q1, Q2;
         Q3, Q4];

% sigma_beta = 1e-10;              
% Q_beta = (sigma_beta^2) * eye(phi);
% Q(1:phi, 1:phi) = Q_beta;



Q(phi+1:end, phi+1:end) = Q_CLK;

%% 观测噪声 R
R = (sigma_t^2) * eye(2*psi);

%% 初始观测向量 Zk
Zk = zeros(2*psi, 1);

%% 初始化矩阵B
Bmat = zeros(phi+2*N-2);

B_mu_omega = -1 * eye(N-1);
Bmat(phi+N:end ,phi+N:end) =B_mu_omega;

% Bmat = -1 * Lmat;
end
