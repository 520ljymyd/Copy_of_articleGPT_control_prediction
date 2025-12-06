function T_mat_k = T_mat_k(N)
    % N: 节点总数，节点 1 为参考
    % 状态中 offset: [delta_2, ..., delta_N]
    % 状态中 skew  : [omega_2, ..., omega_N]
    
    psi  = N * (N - 1) / 2;      % number of undirected links
    rows = 2 * psi;
    cols = 2 * (N - 1);          % (N-1) offsets + (N-1) skews
    
    T_mat_k = zeros(rows, cols);
    
    row = 0;
    for i = 1:N-1
        for j = i+1:N
            % --- 第一条观测: rho_ij ---
            row = row + 1;
            % +delta_i
            if i ~= 1
                T_mat_k(row, i-1) = T_mat_k(row, i-1) + 1;
            end
            % -delta_j
            if j ~= 1
                T_mat_k(row, j-1) = T_mat_k(row, j-1) - 1;
            end
            
            % --- 第二条观测: rho_ji ---
            row = row + 1;
            % -delta_i
            if i ~= 1
                T_mat_k(row, i-1) = T_mat_k(row, i-1) - 1;
            end
            % +delta_j
            if j ~= 1
                T_mat_k(row, j-1) = T_mat_k(row, j-1) + 1;
            end
            
            % 后 (N-1) 列对应 omega，全保持 0
        end
    end
end
