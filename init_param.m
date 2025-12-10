function param = init_param()

param.delta_th = 5e-7;
param.omega_th = 3.2e-12;

param.H_max    = 2;

param.zeta     = 1/sqrt(2);             % 阻尼比
param.B_PLL    = 0.35;                  % PLL带宽
param.B_FLL    = 0.06;                  % FLL带宽
param.k_PLL    = 1;
param.k_FLL    = 1;


% 输入比例系数
param.Kp_delta = 0.5;
param.Kp_omega = 0.25;

% 时钟参数初始化
param.tau0_PLL = 1;
param.tau1_PLL = 0; 
param.tau2_PLL = 0;
param.tau1_FLL = 0;
param.tau2_FLL = 0;

param.Tc         = 0.02;

end
