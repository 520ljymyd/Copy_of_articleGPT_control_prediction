function X = mds(D, out_dim)
% classical MDS
% 输入：
%   D        : N x N 距离矩阵
%   out_dim  : 输出维度（例如 2 或 3）
% 输出：
%   X        : out_dim x N 的相对坐标

    if nargin < 2
        out_dim = 3;
    end

    N = size(D,1);

    % ---- 1. 距离矩阵对称化并置 0 对角 ----
    D = 0.5*(D + D');
    D(1:N+1:end) = 0;

    % ---- 2. 双中心化 ----
    J = eye(N) - ones(N)/N;
    B = -0.5 * J * (D.^2) * J;
    B = 0.5 * (B + B');   % 数值对称化

    % ---- 3. 特征分解（从大到小）----
    [V, Lambda] = eig(B);
    [lambda_sorted, idx] = sort(diag(Lambda), 'descend');
    V = V(:, idx);

    % ---- 4. 筛选正特征值 ----
    pos_idx = find(lambda_sorted > 0);
    if isempty(pos_idx)
        X = zeros(out_dim, N);
        return;
    end

    use_dim = min(out_dim, numel(pos_idx));
    lambda_sel = lambda_sorted(1:use_dim);
    Vsel       = V(:, 1:use_dim);

    % ---- 5. 坐标恢复（参考代码完全一致）----
    Y = Vsel * diag(sqrt(lambda_sel));    % N x dim

    % ---- 6. 输出格式：dim x N ----
    X = Y.';   % 输出为 dim x N
end
