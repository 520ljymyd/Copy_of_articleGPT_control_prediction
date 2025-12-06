function distance_rel_vec = distance_rel_k(position_rel_k)
% 计算节点间的相对距离，并按"1-2、1-3、…、1-N、2-3、…、(N-1)-N"的顺序输出为向量
% 输入：
%   position_rel_k：节点位置矩阵（3×N，每列对应一个节点的3D坐标）
% 输出：
%   distance_rel_vec：相对距离向量（长度为N*(N-1)/2），顺序为i<j的节点对距离

N = size(position_rel_k, 2);  % 节点数量（N≥2）
psi = N*(N-1)/2;              % 总节点对数量（i<j的情况）
distance_rel_vec = zeros(psi, 1);  % 初始化距离向量

idx = 1;  % 向量索引计数器（跟踪当前存储位置）

% 按顺序遍历所有i<j的节点对
for i = 1:N-1          % i从1到N-1（保证i<j）
    for j = i+1:N      % j从i+1到N（避免重复计算i>j的情况）
        % 计算节点i到j的欧氏距离
        d_ij = norm(position_rel_k(:,i) - position_rel_k(:,j));
        % 按顺序存入向量
        distance_rel_vec(idx) = d_ij;
        idx = idx + 1;  % 索引+1，准备存储下一个距离
    end
end

end