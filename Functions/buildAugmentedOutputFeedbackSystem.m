function [A_aug, B_aug, C_aug, D_aug] = buildAugmentedOutputFeedbackSystem(A, B, C, D, K, L)

    A_aug = [A - B*K,     -B*K;
             L*C,         A - B*K - L*C];

    B_aug = [B;
             zeros(size(B))];

    C_aug = [C, zeros(size(C))];
    D_aug = D;

end
