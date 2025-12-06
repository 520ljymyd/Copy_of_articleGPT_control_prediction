function [offsetInit, skewInit] = offset_skew_init(offset_min, offset_max, skew_min, skew_max, N)

offsetInit = (offset_min + (offset_max-offset_min)*rand(1,N));
offsetInit(1) = 0; % 设定参考节点 1 的时钟偏差为 0

skewInit = (skew_min + (skew_max-skew_min)*rand(1,N)); %初始 skew ~ U(1e-9,1e-8)
skewInit(1) = 0;   % 设定参考节点 1 的时钟频偏为 0

end

