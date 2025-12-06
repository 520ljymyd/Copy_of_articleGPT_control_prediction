function [offset_k, skew_k] = offset_skew_update_prediction_control( ...
    offset_prev, skew_prev, dt, N, q1, q2, mu_delta_prev, mu_omega_prev)

F = [1 dt; 0 1];
Q = [ q1*dt + q2*(dt^3)/3,  q2*(dt^2)/2;
    q2*(dt^2)/2        ,  q2*dt      ];

noise_samples = mvnrnd([0, 0], Q, N)';  % 2xN
epsilon_delta = noise_samples(1,:);
epsilon_omega = noise_samples(2,:);

offset_k = zeros(1,N);
skew_k   = zeros(1,N);

% 参考节点 1 不控制
offset_k(1) = 0;
skew_k(1)   = 0;

for i = 2:N
    x_prev = [offset_prev(i); skew_prev(i)];
    w_i    = [epsilon_delta(i); epsilon_omega(i)];
    x_curr = F * x_prev + w_i ...
        - [mu_delta_prev(i-1); mu_omega_prev(i-1)];
    offset_k(i) = x_curr(1);
    skew_k(i)   = x_curr(2);
end
end
