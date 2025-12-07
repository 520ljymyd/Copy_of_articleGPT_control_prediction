function mu_omega_k = generate_mu_omega(delta_est_k, omega_est_k,dt)
%bandwidths of the PLL and FLL were set to 0.35 and 0.06 Hz
%delta_est_k, omega_est_k表示在第n轮通信中，估计出的时钟offset和skew值

PLL_band = 0.35;
FLL_band = 0.06;
zeta = 1/sqrt(2);
sigma_osc = 1e-11;      % 原论文示例 VCXO 短期稳定度
mu_omega_max = sqrt(1.05^2 - 1) * sigma_osc;



T_loop = dt;
alpha_PLL = exp(-2*pi*PLL_band*T_loop);
alpha_FLL = exp(-2*pi*FLL_band*T_loop);



end

