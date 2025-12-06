function Omega_k = Omega_mat_k(N, K, T_n, Zk)

psi = N*(N-1)/2;
phi = (K+1)*psi;

Omega_k = zeros(2*psi, phi);

row = 1;
col = 1;

for i = 1:N-1
    for j = i+1:N

        % 第一条观测：rho_ij → z_ij = rho_ij/c = Zk(row)
        t1 = T_n.^(0:K);

        % 第二条观测：rho_ji → z_ji = rho_ji/c = Zk(row+1)
        z_ji = Zk(row+1);     % 注意：row+1 是这条链路的 z_ji
        t2   = (T_n + z_ji).^(0:K);

        Omega_k(row  , col:col+K) = t1;
        Omega_k(row+1, col:col+K) = t2;

        row = row + 2;
        col = col + (K + 1);
    end
end

end
