function [pos_est_hist, RAMSE_pos_history] = mds_localization_RAMSE( ...
        history_dij_est, pos_hist, ref_idx)

    if nargin < 3
        ref_idx = 1;
    end

    [N1, N2, Nsteps] = size(history_dij_est);
    [dim_pos, N3, Nsteps_pos] = size(pos_hist);
    if N1~=N2 || dim_pos~=3 || N3~=N1 || Nsteps_pos~=Nsteps
        error('尺寸不匹配');
    end
    N = N1;

    pos_est_hist      = zeros(3, N, Nsteps);
    RAMSE_pos_history = zeros(1, Nsteps);

    all_idx = 1:N;
    err_idx = setdiff(all_idx, ref_idx);

    X_prev = [];   % 上一帧，防止镜像跳变

    for k = 1:Nsteps

        Dk = history_dij_est(:,:,k);
        if all(Dk(:) == 0)
            continue;
        end

        % ---- 1) MDS 得到相对坐标（dim x N）----
        X_rel = mds(Dk, 3);

        P_true = squeeze(pos_hist(:,:,k));   % 3 x N

        % ---- 2) 不带反射对齐 (R+)
        [~, Zp, tp] = procrustes(P_true.', X_rel.', ...
                                 'Scaling', true, 'Reflection', false);
        X_plus  = Zp.';   

        % ---- 3) 允许反射的对齐 (R-)
        [~, Zm, tm] = procrustes(P_true.', X_rel.', ...
                                 'Scaling', true, 'Reflection', true);
        X_minus = Zm.';   

        % ---- 4) 选择更连续的解（防跳变）----
        if isempty(X_prev)
            % 第一帧：选误差更小的
            if norm(X_plus(:)-P_true(:)) <= norm(X_minus(:)-P_true(:))
                X_aligned = X_plus;
            else
                X_aligned = X_minus;
            end
        else
            % 后续帧：选和上一帧更接近的解
            if norm(X_plus(:)-X_prev(:)) <= norm(X_minus(:)-X_prev(:))
                X_aligned = X_plus;
            else
                X_aligned = X_minus;
            end
        end

        X_prev = X_aligned;
        pos_est_hist(:,:,k) = X_aligned;

        % ---- 5) 计算位置误差 ----
        diff_vec = X_aligned - P_true;
        err_norm = sqrt(sum(diff_vec.^2,1));
        err_norm = err_norm - mean(err_norm);  
        RAMSE_pos_history(k) = sqrt(mean(err_norm(err_idx).^2)); 
    end
end
