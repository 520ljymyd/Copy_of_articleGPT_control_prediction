function [Xkk_1, Hkk_1] = kf_predict(L, Q, Xk_1, Hk_1, B, Lambda_prev)
    Xkk_1 = L * Xk_1 + B * Lambda_prev;
    Hkk_1 = L * Hk_1 * L' + Q;
end