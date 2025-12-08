function [K, e_cl] = designLQR(A, B, C)

    disp("Part 7 – LQR Design");

    Q = C' * C;
    R = 0.1 * eye(size(B,2));

    [K, ~, e_cl] = lqr(A, B, Q, R);

    disp("LQR Gain K:");
    disp(K);

    disp("Closed-loop poles:");
    disp(e_cl);

end
